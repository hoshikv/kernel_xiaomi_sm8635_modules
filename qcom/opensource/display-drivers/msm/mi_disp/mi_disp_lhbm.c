/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020 XiaoMi, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "mi-disp-lhbm:[%s:%d] " fmt, __func__, __LINE__
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <uapi/linux/sched/types.h>

#include "dsi_display.h"
#include "sde_trace.h"

#include "mi_disp_feature.h"
#include "mi_disp_lhbm.h"
#include "mi_disp_print.h"
#include "mi_dsi_display.h"
#include "mi_dsi_panel.h"
#include "mi_panel_id.h"
#include "sde_vm.h"

static struct disp_lhbm_fod *g_lhbm_fod[MI_DISP_MAX];

/* hoshikv-fod: inbuilt FOD-HBM watch state (driver-side fod_press_status consumer) */
static struct task_struct *g_fod_watch_thread = NULL;
static atomic_t g_fod_watch_enable = ATOMIC_INIT(0);
static int g_fod_watch_disp_id = MI_DISP_PRIMARY;
/* hoshikv-fod: current press state of the watch. Promoted to a global so
 * fod_watch_enable(0) can force LHBM-OFF without emitting a finger-up event
 * (keeps fod anim running on v()), and so a fresh press edge works after a
 * re-enable. */
static bool g_fod_watch_pressed = false;
/* hoshikv-fod: 0.3s auto-off timer for the FOD HBM "light". Used so a press
 * naturally dims the light even if the finger is held, independent of release.
 * Goes through the SILENT set_disp_param path (no onFpTouch/Ui event) so the
 * fod anim is unaffected. */
static struct delayed_work g_fod_hbm_auto_off;
/* hoshikv-fod: non-consuming fod touch state for the lib's fod anim.
 * The driver is the sole reader of fod_press_status (one-shot sysfs).
 * On press/release it updates g_fod_touch_state and calls sysfs_notify so
 * the lib can poll() for the state change. This avoids the unreliable
 * MI_DISP_EVENT_FOD / allow_tx_lhbm gate and keeps both HBM and anim
 * driven from the same fod_press_status source. */
static atomic_t g_fod_touch_state = ATOMIC_INIT(0);
/* hoshikv-doze: driver-owned "new doze" state. If xiaomi's doze_brightness flow
 * left panel_state=ON (not genuinely marked as AOD/doze), the FOD-HBM HLPM gate
 * refuses to light. So the inbuilt FOD service establishes the doze context
 * itself: on FOD-HBM-ON it force-sets panel_state=DOZE_HIGH (making
 * is_aod_and_panel_initialized() true) so the HLPM command path is selected,
 * and on FOD-HBM-OFF it restores the saved prior state. */
static int g_fod_forced_doze = 0;
static int g_fod_forced_doze_prev = PANEL_STATE_ON;

static int mi_disp_lhbm_fod_thread_fn(void *arg);

bool is_local_hbm(int disp_id) {
  struct dsi_display *display = NULL;

  if (is_support_disp_id(disp_id)) {
    if (disp_id == MI_DISP_PRIMARY)
      display = mi_get_primary_dsi_display();
    else
      display = mi_get_secondary_dsi_display();

    if (display && display->panel)
      return display->panel->mi_cfg.local_hbm_enabled;
    else
      return false;
  } else {
    DISP_ERROR("unknown display id\n");
    return false;
  }
}

bool mi_disp_lhbm_fod_enabled(struct dsi_panel *panel) {
  return panel ? panel->mi_cfg.local_hbm_enabled : false;
}

int mi_disp_lhbm_fod_thread_create(struct disp_feature *df, int disp_id) {
  int ret = 0;
  struct dsi_display *display = NULL;
  struct disp_lhbm_fod *lhbm_fod = NULL;

  if (!df || !is_support_disp_id(disp_id)) {
    DISP_ERROR("invalid params\n");
    return -EINVAL;
  }

  if (!df->d_display[disp_id].display ||
      df->d_display[disp_id].intf_type != MI_INTF_DSI) {
    DISP_ERROR("unsupported display(%s intf)\n",
               get_disp_intf_type_name(df->d_display[disp_id].intf_type));
    return -EINVAL;
  }

  display = (struct dsi_display *)df->d_display[disp_id].display;
  if (!mi_disp_lhbm_fod_enabled(display->panel)) {
    DISP_INFO("%s panel is not local hbm\n", get_disp_id_name(disp_id));
    return 0;
  }

  lhbm_fod = kzalloc(sizeof(struct disp_lhbm_fod), GFP_KERNEL);
  if (!lhbm_fod) {
    DISP_ERROR("can not allocate buffer for disp_lhbm\n");
    return -ENOMEM;
  }

  df->d_display[disp_id].lhbm_fod_ptr = lhbm_fod;
  lhbm_fod->display = display;

  INIT_LIST_HEAD(&lhbm_fod->event_list);
  spin_lock_init(&lhbm_fod->spinlock);

  atomic_set(&lhbm_fod->allow_tx_lhbm, 0);
  atomic_set(&lhbm_fod->target_brightness,
             LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);

  init_waitqueue_head(&lhbm_fod->fod_pending_wq);

  lhbm_fod->fod_thread = kthread_run(mi_disp_lhbm_fod_thread_fn, lhbm_fod,
                                     "disp_lhbm_fod:%d", disp_id);
  if (IS_ERR(lhbm_fod->fod_thread)) {
    DISP_ERROR("failed to create disp_fod:%d kthread\n", disp_id);
    ret = PTR_ERR(lhbm_fod->fod_thread);
    lhbm_fod->fod_thread = NULL;
    goto error;
  }
  /* set realtime priority */
  sched_set_fifo(lhbm_fod->fod_thread);

  g_lhbm_fod[disp_id] = lhbm_fod;

  DISP_INFO("create disp_lhbm_fod:%d kthread success\n", disp_id);

	/* hoshikv-fod: start the inbuilt fod_press_status watch alongside */
	mi_disp_lhbm_fod_watch_create(df, disp_id);

	return ret;

error:
  kfree(lhbm_fod);
  return ret;
}

int mi_disp_lhbm_fod_thread_destroy(struct disp_feature *df, int disp_id) {
  int ret = 0;
  struct disp_lhbm_fod *lhbm_fod = NULL;

  if (!df || !is_support_disp_id(disp_id)) {
    DISP_ERROR("invalid params\n");
    return -EINVAL;
  }

  lhbm_fod = df->d_display[disp_id].lhbm_fod_ptr;
  if (lhbm_fod) {
    if (lhbm_fod->fod_thread) {
      kthread_stop(lhbm_fod->fod_thread);
      lhbm_fod->fod_thread = NULL;
    }
    kfree(lhbm_fod);
  }

  df->d_display[disp_id].lhbm_fod_ptr = NULL;
  g_lhbm_fod[disp_id] = NULL;

	/* hoshikv-fod: stop the inbuilt fod_press_status watch */
	mi_disp_lhbm_fod_watch_destroy(df, disp_id);

	DISP_INFO("destroy disp_lhbm_fod:%d kthread success\n", disp_id);

  return ret;
}

struct disp_lhbm_fod *mi_get_disp_lhbm_fod(int disp_id) {
  if (is_support_disp_id(disp_id)) {
    return g_lhbm_fod[disp_id];
  } else {
    DISP_ERROR("unknown display id\n");
    return NULL;
  }
}

int mi_disp_lhbm_fod_allow_tx_lhbm(struct dsi_display *display, bool enable) {
  struct disp_lhbm_fod *lhbm_fod = NULL;

  if (!display) {
    DISP_ERROR("Invalid display ptr\n");
    return -EINVAL;
  }

  if (!mi_disp_lhbm_fod_enabled(display->panel)) {
    DISP_DEBUG("%s panel is not local hbm\n", display->display_type);
    return 0;
  }

  lhbm_fod = mi_get_disp_lhbm_fod(mi_get_disp_id(display->display_type));
  if (!lhbm_fod) {
    DISP_ERROR("Invalid lhbm_fod ptr\n");
    return -EINVAL;
  }

  if (lhbm_fod->display == display &&
      atomic_read(&lhbm_fod->allow_tx_lhbm) != enable) {
    atomic_set(&lhbm_fod->allow_tx_lhbm, enable);
    DISP_INFO("%s display allow_tx_lhbm = %d\n", display->display_type, enable);
    if (enable) {
      wake_up_interruptible(&lhbm_fod->fod_pending_wq);
      /* if target_brightness is saved, will restor the local hbm on */
      if (atomic_read(&lhbm_fod->disp_off_target_brightness) !=
              LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
          atomic_read(&lhbm_fod->disp_off_target_brightness) !=
              LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
        mi_disp_set_local_hbm(
            mi_get_disp_id(display->display_type),
            atomic_read(&lhbm_fod->disp_off_target_brightness));
      }
      DISP_INFO("%s display wake up local disp_fod kthread\n",
                display->display_type);
    } else {
      /* If the last fod fingerprint status is that the finger is pressed and
       * the finger is not lifted when display off, save target_brightness */
      if (atomic_read(&lhbm_fod->target_brightness) !=
              LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
          atomic_read(&lhbm_fod->target_brightness) !=
              LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP &&
          list_empty(&lhbm_fod->event_list)) {
        atomic_set(&lhbm_fod->disp_off_target_brightness,
                   atomic_read(&lhbm_fod->target_brightness));
      } else {
        atomic_set(&lhbm_fod->disp_off_target_brightness,
                   LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
      }
      atomic_set(&lhbm_fod->target_brightness,
                 LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
    }
  }
  return 0;
}

int mi_disp_lhbm_fod_update_layer_state(struct dsi_display *display,
                                        struct mi_layer_flags flags) {
  struct disp_lhbm_fod *lhbm_fod = NULL;

  if (!display) {
    DISP_ERROR("Invalid display ptr\n");
    return -EINVAL;
  }

  if (!mi_disp_lhbm_fod_enabled(display->panel)) {
    DISP_DEBUG("%s panel is not local hbm\n", display->display_type);
    return 0;
  }

  lhbm_fod = mi_get_disp_lhbm_fod(mi_get_disp_id(display->display_type));
  if (!lhbm_fod) {
    DISP_ERROR("Invalid lhbm_fod ptr\n");
    return -EINVAL;
  }

  spin_lock(&lhbm_fod->spinlock);
  lhbm_fod->layer_flags = flags;
  spin_unlock(&lhbm_fod->spinlock);

  return 0;
}

static int mi_disp_lhbm_fod_event_notify(struct disp_lhbm_fod *lhbm_fod, int fod_status)
{
	struct dsi_display *display = NULL;
	int disp_id = MI_DISP_PRIMARY;
	u32 fod_ui_ready = 0, refresh_rate = 0;
	u32 ui_ready_delay_frame = 0;
	u64 delay_us = 0;

	if (!lhbm_fod || !lhbm_fod->display) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	display = lhbm_fod->display;

	if (!display->panel || !display->panel->cur_mode){
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;
	if (fod_status == FOD_EVENT_FPS && refresh_rate == 30) {
		fod_ui_ready = LOCAL_HBM_NEED_UPDATE_TO_FOD_FPS;
		mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_FOD,
				sizeof(fod_ui_ready), fod_ui_ready);
		return -NEED_UPDATE_TO_FOD_FPS;
	}

	if (fod_status == FOD_EVENT_DOWN &&
		display->panel->mi_cfg.lhbm_ui_ready_delay_frame > 0) {
		ui_ready_delay_frame = display->panel->mi_cfg.lhbm_ui_ready_delay_frame;
		delay_us = 1000000 / refresh_rate * ui_ready_delay_frame;
		DISP_INFO("refresh_rate(%d), delay (%d) frame, delay_us(%llu)\n",
				refresh_rate, ui_ready_delay_frame, delay_us);
		usleep_range(delay_us, delay_us + 10);
	}

	if (fod_status == FOD_EVENT_DOWN) {
		if (atomic_read(&lhbm_fod->target_brightness) == LHBM_TARGET_BRIGHTNESS_WHITE_110NIT)
			fod_ui_ready = LOCAL_HBM_UI_READY | FOD_LOW_BRIGHTNESS_CAPTURE;
		else
			fod_ui_ready = LOCAL_HBM_UI_READY;
	} else {
		fod_ui_ready = LOCAL_HBM_UI_NONE;
	}

	if (atomic_read(&lhbm_fod->allow_tx_lhbm)) {
		disp_id = mi_get_disp_id(display->display_type);
		mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_FOD,
				sizeof(fod_ui_ready), fod_ui_ready);

		DISP_INFO("%s display fod_ui_ready notify=%d\n",
			display->display_type, fod_ui_ready);
	} else {
		pr_err("hoshikv-fod: MI_DISP_EVENT_FOD NOT emitted fid=%d allow_tx=%d\n",
			fod_ui_ready, atomic_read(&lhbm_fod->allow_tx_lhbm));
	}

	return 0;
}

static int mi_disp_lhbm_fod_set_disp_param(struct disp_lhbm_fod *lhbm_fod, u32 lhbm_value)
{
	struct dsi_panel *panel = NULL;
	struct mi_dsi_panel_cfg *mi_cfg = NULL;
	struct disp_feature_ctl ctl;
	struct dsi_display *dsi_display = NULL;
	struct sde_kms *sde_kms = NULL;
	int rc = 0;

	if (!lhbm_fod || !lhbm_fod->display || !lhbm_fod->display->panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}
	dsi_display = lhbm_fod->display;
	memset(&ctl, 0, sizeof(struct disp_feature_ctl));
	panel = lhbm_fod->display->panel;

	if (sde_kms_is_suspend_blocked(dsi_display->drm_dev) &&
	    !is_aod_and_panel_initialized(panel)) {
		DISP_ERROR("sde_kms is suspended, skip to set disp_param\n");
		return -EBUSY;
	}

	if (panel->power_mode != SDE_MODE_DPMS_ON &&
	    panel->power_mode != SDE_MODE_DPMS_LP1 &&
	    panel->power_mode != SDE_MODE_DPMS_LP2) {
		pr_info("hoshikv-fod: panel power_mode=%d not stable, skip HBM inj\n",
			panel->power_mode);
		return -EINVAL;
	}

	if (!atomic_read(&lhbm_fod->allow_tx_lhbm)) {
		pr_info("hoshikv-fod: allow_tx_lhbm=0, skip HBM inj\n");
		return -EINVAL;
	}

	sde_kms = dsi_display_get_kms(dsi_display);
	if (sde_kms) {
		sde_vm_lock(sde_kms);
		if (!sde_vm_owns_hw(sde_kms)) {
			DISP_ERROR("op not supported due to HW unavailablity\n");
			rc = -EOPNOTSUPP;
			goto end;
		}
	}

	/* hoshikv-fod (single-flow): serialize the FOD-HBM command against the panel
	 * doze/power transition. dsi_display_set_power() holds doze_lock while it
	 * brings the panel into doze (dpms ON->LP1/LP2) and also acquires panel_lock
	 * internally (order: doze_lock -> panel_lock). We take doze_lock BEFORE
	 * panel_lock so, when the user touches FOD while the screen is off, our HBM
	 * command BLOCKS until the doze entry fully completes and the link is stable.
	 * Only then do we send HBM ON/OFF over DSI — this is what removes the race
	 * that wedged the DMA queue and blanked the display. */
	mutex_lock(&panel->mi_cfg.doze_lock);
	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;

	switch (lhbm_value) {
	case LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT:
	case LHBM_TARGET_BRIGHTNESS_WHITE_110NIT:
	case LHBM_TARGET_BRIGHTNESS_GREEN_500NIT:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		/* hoshikv-doze: establish the doze/new-AOD context for FOD-HBM on -- see
		 * header comment on g_fod_forced_doze. Runs under panel_lock. Only when
		 * the panel is GENUINELY in DPMS AOD (power_mode LP1/LP2) but xiaomi's
		 * doze_brightness left panel_state not marked as DOZE; NEVER when the
		 * display is normally on (lockscreen/enroll -> power_mode DPMS_ON), so
		 * FOD-HBM there keeps the NORMAL (non-HLPM) path and brightness is
		 * untouched. */
		if ((panel->power_mode == SDE_MODE_DPMS_LP1 ||
		     panel->power_mode == SDE_MODE_DPMS_LP2) &&
		    mi_cfg->panel_state != PANEL_STATE_DOZE_HIGH &&
		    mi_cfg->panel_state != PANEL_STATE_DOZE_LOW &&
		    atomic_read(&g_fod_watch_enable)) {
			g_fod_forced_doze_prev = mi_cfg->panel_state;
			mi_cfg->panel_state = PANEL_STATE_DOZE_HIGH;
			g_fod_forced_doze = 1;
			pr_info("hoshikv-fod: NEWDOZE forced panel_state=DOZE_HIGH (was %d)\n",
				g_fod_forced_doze_prev);
		}
		if (lhbm_value == LHBM_TARGET_BRIGHTNESS_GREEN_500NIT) {
			ctl.feature_val = LOCAL_HBM_NORMAL_GREEN_500NIT;
		} else if (lhbm_value == LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT) {
			if (is_aod_and_panel_initialized(panel) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW)) {
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_1000NIT;
				pr_info("hoshikv-fod: LHBMPICK=HLPM aod=%d state=%d\n",
					is_aod_and_panel_initialized(panel), mi_cfg->panel_state);
			} else {
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_1000NIT;
				pr_info("hoshikv-fod: LHBMPICK=NORMAL aod=%d state=%d\n",
					is_aod_and_panel_initialized(panel), mi_cfg->panel_state);
			}
		} else if (lhbm_value == LHBM_TARGET_BRIGHTNESS_WHITE_110NIT) {
			if (is_aod_and_panel_initialized(panel) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW))
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_110NIT;
			else
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_110NIT;
		} else {
			DISP_ERROR("invalid target_brightness = %d\n", lhbm_fod->target_brightness);
		}
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		/* hoshikv-doze: undo the forced doze context on FOD-HBM off. */
		if (g_fod_forced_doze) {
			mi_cfg->panel_state = g_fod_forced_doze_prev;
			g_fod_forced_doze = 0;
			pr_info("hoshikv-fod: NEWDOZE restored panel_state=%d\n",
				mi_cfg->panel_state);
		}
		if (is_aod_and_panel_initialized(panel))
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT_RESTORE;
		else
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL;
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		/* hoshikv-doze: undo the forced doze context on FOD-HBM off. */
		if (g_fod_forced_doze) {
			mi_cfg->panel_state = g_fod_forced_doze_prev;
			g_fod_forced_doze = 0;
			pr_info("hoshikv-fod: NEWDOZE restored panel_state=%d\n",
				mi_cfg->panel_state);
		}
		if (is_aod_and_panel_initialized(panel)) {
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT;
		} else {
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL;
		}
		break;
	default:
		break;
	}

	atomic_set(&lhbm_fod->target_brightness, lhbm_value);
	rc = mi_dsi_panel_set_lhbm_fod_locked(panel, &ctl);
	mi_cfg->feature_val[DISP_FEATURE_LOCAL_HBM] = ctl.feature_val;
	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&panel->mi_cfg.doze_lock);

end:
  if (sde_kms)
    sde_vm_unlock(sde_kms);

  return rc;
}

int mi_disp_lhbm_aod_to_normal_optimize(struct dsi_display *display,
                                        bool enable) {
  struct disp_feature_ctl ctl;
  int rc = 0;

  if (!display || !display->panel) {
    DISP_ERROR("invalid params\n");
    return -EINVAL;
  }

  if (!display->panel->mi_cfg.need_fod_animal_in_normal)
    return 0;

  memset(&ctl, 0, sizeof(struct disp_feature_ctl));
  ctl.feature_id = DISP_FEATURE_AOD_TO_NORMAL;
  ctl.feature_val = enable ? FEATURE_ON : FEATURE_OFF;

  rc = mi_dsi_display_set_disp_param(display, &ctl);

  return rc;
}

static bool
mi_disp_lhbm_fod_thread_should_wake(struct disp_lhbm_fod *lhbm_fod) {
  bool should_wake = false;
  unsigned long flags;

  spin_lock_irqsave(&lhbm_fod->spinlock, flags);

  if (list_empty(&lhbm_fod->event_list) ||
      !atomic_read(&lhbm_fod->allow_tx_lhbm))
    should_wake = false;
  else
    should_wake = true;

  spin_unlock_irqrestore(&lhbm_fod->spinlock, flags);

  return should_wake;
}

static int mi_disp_lhbm_fod_thread_fn(void *arg) {
  int rc = 0;
  struct disp_lhbm_fod *lhbm_fod = (struct disp_lhbm_fod *)arg;
  struct lhbm_setting *entry = NULL, *temp = NULL;
  struct lhbm_setting lhbm_setting_event;
  unsigned long flag = 0;

  while (!kthread_should_stop()) {
    rc =
        wait_event_interruptible(lhbm_fod->fod_pending_wq,
                                 mi_disp_lhbm_fod_thread_should_wake(lhbm_fod));
    if (rc) {
      /* Some event woke us up */
      DISP_WARN("wait_event_interruptible rc = %d\n", rc);
      continue;
    }

    spin_lock_irqsave(&lhbm_fod->spinlock, flag);
    entry = list_last_entry(&lhbm_fod->event_list, struct lhbm_setting, link);
    DISP_INFO("lhbm_value(%d)\n", entry->lhbm_value);
    memcpy(&lhbm_setting_event, entry, sizeof(lhbm_setting_event));
    if ((mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) ==
             N16T_PANEL_PA ||
         mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) ==
             N16T_PANEL_PB ||
         mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) ==
             N16T_PANEL_PC ||
         mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) ==
             O16U_PANEL_PA ||
         mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) ==
             O16U_PANEL_PB) &&
        is_aod_and_panel_initialized(lhbm_fod->display->panel)) {
      /* Notify switch to fod fps */
      if (lhbm_setting_event.lhbm_value !=
              LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
          lhbm_setting_event.lhbm_value !=
              LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
        rc = mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_FPS);
        if (rc == -NEED_UPDATE_TO_FOD_FPS) {
          mi_disp_lhbm_fod_allow_tx_lhbm(lhbm_fod->display, false);
          DISP_INFO("Stop to allow tx lhbm, wait to swtich fod fps!");
          spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);
          continue;
        }
      }
    }
    list_for_each_entry_safe(entry, temp, &lhbm_fod->event_list, link) {
      DISP_DEBUG("in list, lhbm_value(%d)\n", entry->lhbm_value);
      list_del(&entry->link);
      kfree(entry);
    }
    if (atomic_read(&lhbm_fod->target_brightness) !=
        lhbm_setting_event.lhbm_value) {
      atomic_set(&lhbm_fod->target_brightness, lhbm_setting_event.lhbm_value);

      spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);

      if (lhbm_setting_event.lhbm_value ==
              LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP ||
          lhbm_setting_event.lhbm_value ==
              LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
        mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_UP);
      }

      rc = mi_disp_lhbm_fod_set_disp_param(lhbm_fod,
                                           lhbm_setting_event.lhbm_value);
      if (rc) {
        DISP_ERROR("lhbm_fod failed to set_disp_param, rc = %d\n", rc);
      } else if (lhbm_setting_event.lhbm_value !=
                     LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
                 lhbm_setting_event.lhbm_value !=
                     LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
        mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_DOWN);
      }
    } else {
      spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);
      DISP_INFO("same lhbm setting event: %d, return\n",
                lhbm_setting_event.lhbm_value);
    }
  }

  return 0;
}

int mi_disp_set_local_hbm(int disp_id, int lhbm_value) {
  struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);
  struct lhbm_setting *lhbm_setting_event = NULL, *entry = NULL;
  unsigned long flags;
  int rc = 0;

#ifdef CONFIG_FACTORY_BUILD
  return 0;
#endif

  if (!is_local_hbm(disp_id)) {
    DISP_DEBUG("%s panel is not local hbm\n", get_disp_id_name(disp_id));
    return 0;
  }

  if (!lhbm_fod) {
    DISP_ERROR("invalid lhbm_fod ptr\n");
    return -EINVAL;
  }

  spin_lock_irqsave(&lhbm_fod->spinlock, flags);

  mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_LOCAL_HBM_VALUE,
                                       sizeof(lhbm_value), lhbm_value);

  if (lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP ||
      lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
    atomic_set(&lhbm_fod->disp_off_target_brightness, lhbm_value);
  }

  if (atomic_read(&lhbm_fod->target_brightness) ==
          LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
      lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP &&
      list_empty(&lhbm_fod->event_list)) {
    DISP_INFO("skip set LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP\n");
    goto exit;
  }

  lhbm_setting_event = kzalloc(sizeof(struct lhbm_setting), GFP_ATOMIC);
  if (!lhbm_setting_event) {
    DISP_ERROR("failed to allocate memory for lhbm_setting_event\n");
    rc = ENOMEM;
    goto exit;
  }

  lhbm_setting_event->lhbm_value = lhbm_value;
  INIT_LIST_HEAD(&lhbm_setting_event->link);
  list_add_tail(&lhbm_setting_event->link, &lhbm_fod->event_list);

  list_for_each_entry(entry, &lhbm_fod->event_list, link) {
    DISP_DEBUG("in list, lhbm_value(%d)\n", entry->lhbm_value);
  }

	pr_info("hoshikv-fod: local_hbm_value:%s(0x%x) queued\n",
		get_lhbm_value_name(lhbm_value), lhbm_value);
	wake_up_interruptible(&lhbm_fod->fod_pending_wq);

exit:
  spin_unlock_irqrestore(&lhbm_fod->spinlock, flags);
  return rc;
}

int mi_disp_update_0size_lhbm_info(struct dsi_panel *panel) {
  struct mi_dsi_panel_cfg *mi_cfg = NULL;
  struct disp_feature_ctl ctl;
  int rc = 0;

  if (!panel) {
    DISP_ERROR("invalid params\n");
    return -EINVAL;
  }

  if (mi_get_panel_id_by_dsi_panel(panel) != N3_PANEL_PA)
    return rc;

  mi_cfg = &panel->mi_cfg;

  if (is_hbm_fod_on(panel)) {
    DISP_DEBUG("skip 0 size lhbm due to lhbm is on\n");
    return rc;
  }
  if (panel->power_mode == SDE_MODE_DPMS_ON) {
    memset(&ctl, 0, sizeof(struct disp_feature_ctl));
    if (mi_cfg->lhbm_gxzw && !mi_cfg->lhbm_0size_on &&
        mi_cfg->feature_val[DISP_FEATURE_FP_STATUS] != AUTH_STOP) {
      if (mi_cfg->last_bl_level) {
        rc = mi_dsi_panel_set_lhbm_0size_locked(panel);
      }
      mi_cfg->lhbm_0size_on = true;
      DISP_DEBUG("gxzw apper,0 size lhbm on,last_bl_level[%d]\n",
                 mi_cfg->last_bl_level);
    } else if (!mi_cfg->lhbm_gxzw && mi_cfg->lhbm_0size_on) {
      ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
      ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT;
      rc = mi_dsi_panel_set_lhbm_fod_locked(panel, &ctl);
      panel->mi_cfg.feature_val[DISP_FEATURE_LOCAL_HBM] = ctl.feature_val;
      mi_cfg->lhbm_0size_on = false;
      DISP_DEBUG("gxzw quit,0 size lhbm off to normal,last_bl_level[%d]\n",
                 mi_cfg->last_bl_level);
    }
  }

  return rc;
}

int mi_disp_update_0size_lhbm_layer(struct dsi_display *dsi_display,
                                    u32 mi_gxzw_flags) {
  struct dsi_panel *panel = NULL;
  struct mi_dsi_panel_cfg *mi_cfg = NULL;
  int rc = 0;

  if (!dsi_display || !dsi_display->panel) {
    DISP_ERROR("invalid params\n");
    return -EINVAL;
  }

  panel = dsi_display->panel;

  if (mi_get_panel_id_by_dsi_panel(panel) != N3_PANEL_PA)
    return rc;

  mi_cfg = &panel->mi_cfg;
  dsi_panel_acquire_panel_lock(panel);
  rc = mi_disp_update_0size_lhbm_info(panel);
  dsi_panel_release_panel_lock(panel);

  return rc;
}

/*
 * hoshikv-fod: fan-out emitter. On a press edge this drives onFpTouch
 * (fod animation) DIRECTLY, independent of the local-HBM / FOD-service engine
 * and its allow_tx_lhbm gate. That is what makes "kadang fodanim gak
 * ketrigger" go away: previously the anim only fired inside the HBM path and
 * was silently dropped when allow_tx_lhbm got stuck false. Now the two
 * branches are decoupled (single read -> HBM branch + anim branch).
 */
static void mi_disp_lhbm_fod_watch_emit(int disp_id, bool down)
{
	struct dsi_display *display = NULL;
	u32 fod_ui_ready = down ? LOCAL_HBM_UI_READY : LOCAL_HBM_UI_NONE;
	struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);

	if (lhbm_fod && lhbm_fod->display)
		display = lhbm_fod->display;

	mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_FOD,
			sizeof(fod_ui_ready), fod_ui_ready);
	pr_info("hoshikv-fod: watch emit onFpTouch=%d fid=%u dspl=%s\n",
			down ? 1 : 0, fod_ui_ready,
			display ? display->display_type : "none");
}

/*
 * hoshikv-fod: 0.3s auto-off of the FOD HBM light. Silent (set_disp_param
 * directly, no MI_DISP_EVENT_FOD) so the fod anim stays untouched while the
 * high-brightness light itself dims back after 300ms for a natural look.
 */
static void hoshikv_fod_hbm_auto_off_handler(struct work_struct *work)
{
	struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(g_fod_watch_disp_id);

	if (lhbm_fod)
		mi_disp_lhbm_fod_set_disp_param(lhbm_fod,
				LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
	pr_info("hoshikv-fod: HBM 0.3s auto-off done\n");
}

/*
 * hoshikv-fod: non-consuming sysfs state for fod anim.
 * The lib polls this node (POLLPRI) and reads 0/1 for finger up/down.
 */
int mi_disp_lhbm_fod_get_touch_state(void)
{
	return atomic_read(&g_fod_touch_state);
}

static void hoshikv_fod_sysfs_notify(void)
{
	struct disp_feature *df = mi_get_disp_feature();
	struct disp_display *dd_ptr;

	if (!df)
		return;
	dd_ptr = &df->d_display[g_fod_watch_disp_id];
	if (dd_ptr && dd_ptr->dev)
		sysfs_notify(&dd_ptr->dev->kobj, NULL, "hoshikv_fod_state");
}

/*
 * hoshikv-fod: inbuilt FOD-HBM watch kthread.
 *
 * While enabled, this is the SINGLE kernel-side consumer of fod_press_status.
 * On press it enqueues local-HBM-ON (which, thanks to the doze-HLPM fix, selects
 * the HLPM command when the panel is in doze and lights the FOD area), and on
 * release it enqueues local-HBM-OFF. The existing FOD thread emits
 * MI_DISP_EVENT_FOD (UI_READY / NONE) to the registered client, which is the
 * driver->lib callback the lib turns into onFpTouch animation. No double-read:
 * the lib no longer reads fod_press_status itself.
 */
static int mi_disp_lhbm_fod_watch_thread_fn(void *arg)
{
	struct file *fp = NULL;
	struct poll_wqueues table;
	__poll_t mask;
	char buf[8] = {0};
	ssize_t n;
	char last_raw = '\0';

	fp = filp_open(FOD_PRESS_STATUS_NODE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("hoshikv-fod: open %s failed, err=%ld\n",
			FOD_PRESS_STATUS_NODE, PTR_ERR(fp));
		fp = NULL;
	} else {
		pr_info("hoshikv-fod: fod_press_status node opened OK\n");
	}

	/* register this thread once on the node's kernfs poll waitqueue; the
	 * entry persists until poll_freewait, so it accumulates nothing. */
	poll_initwait(&table);
	if (fp && fp->f_op && fp->f_op->poll)
		fp->f_op->poll(fp, &table.pt);

	while (!kthread_should_stop()) {
		/* block here until sysfs_notify wakes us; re-check level without
		 * re-registering (NULL table) so nothing is accumulated. */
		set_current_state(TASK_INTERRUPTIBLE);
		mask = 0;
		if (fp && fp->f_op && fp->f_op->poll)
			mask |= fp->f_op->poll(fp, NULL);
		if (kthread_should_stop())
			break;
		if (!(mask & (EPOLLPRI | EPOLLERR)))
			schedule();
		__set_current_state(TASK_RUNNING);

		if (kthread_should_stop())
			break;

		if (fp) {
			/* ALWAYS consume the sysfs value, even while disabled: the
			 * read is what syncs kernfs of->event to on->event. If we only
			 * read while enabled, a fod_press_status notify arriving while
			 * disabled leaves kernfs poll() permanently ready (EPOLLPRI),
			 * so this loop never hits schedule() again and the kthread
			 * spins at 100% CPU -> battery drain. Reading is cheap and
			 * purely event-driven; only the actions are gated on enable.
			 */
			fp->f_pos = 0;
			memset(buf, 0, sizeof(buf));
			n = kernel_read(fp, buf, sizeof(buf) - 1, &fp->f_pos);
			if (n > 0 && atomic_read(&g_fod_watch_enable)) {
				if (buf[0] != last_raw) {
					pr_info("hoshikv-fod: raw value=%c n=%d (en=%d)\n",
						buf[0], n, atomic_read(&g_fod_watch_enable));
					last_raw = buf[0];
				}
				if (buf[0] == '1' && !g_fod_watch_pressed) {
					g_fod_watch_pressed = true;
					atomic_set(&g_fod_touch_state, 1);
					hoshikv_fod_sysfs_notify();
					pr_info("hoshikv-fod: press detected\n");
					/* branch A: FOD service (HBM light), silent direct
					 * command -> no MI_DISP_EVENT_FOD here */
					if (mi_get_disp_lhbm_fod(g_fod_watch_disp_id))
						mi_disp_lhbm_fod_set_disp_param(
							mi_get_disp_lhbm_fod(g_fod_watch_disp_id),
							LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT);
					/* branch B: fod animation (onFpTouch), independent of
					 * HBM/allow_tx gate -> never dropped */
					mi_disp_lhbm_fod_watch_emit(g_fod_watch_disp_id, true);
					/* natural: auto-dim the HBM light after 0.2s */
					schedule_delayed_work(&g_fod_hbm_auto_off,
						msecs_to_jiffies(200));
				} else if (buf[0] == '0' && g_fod_watch_pressed) {
					g_fod_watch_pressed = false;
					cancel_delayed_work_sync(&g_fod_hbm_auto_off);
					pr_info("hoshikv-fod: release detected\n");
					if (mi_get_disp_lhbm_fod(g_fod_watch_disp_id))
						mi_disp_lhbm_fod_set_disp_param(
							mi_get_disp_lhbm_fod(g_fod_watch_disp_id),
							LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
					mi_disp_lhbm_fod_watch_emit(g_fod_watch_disp_id, false);
					atomic_set(&g_fod_touch_state, 0);
					hoshikv_fod_sysfs_notify();
				}
			}
		}
	}

	poll_freewait(&table);

	if (fp)
		filp_close(fp, NULL);

	return 0;
}

int mi_disp_lhbm_fod_watch_create(struct disp_feature *df, int disp_id)
{
	struct dsi_display *display = NULL;

	if (!df || !is_support_disp_id(disp_id)) {
		pr_err("hoshikv-fod: invalid params\n");
		return -EINVAL;
	}

	if (g_fod_watch_thread)
		return 0;

	display = (struct dsi_display *)df->d_display[disp_id].display;
	if (!display || !display->panel) {
		pr_err("hoshikv-fod: no display/panel\n");
		return -EINVAL;
	}

	g_fod_watch_disp_id = disp_id;
	INIT_DELAYED_WORK(&g_fod_hbm_auto_off, hoshikv_fod_hbm_auto_off_handler);
	g_fod_watch_thread = kthread_run(mi_disp_lhbm_fod_watch_thread_fn,
			NULL, "hoshikv_fod_watch:%d", disp_id);
	if (IS_ERR(g_fod_watch_thread)) {
		long err = PTR_ERR(g_fod_watch_thread);
		pr_err("hoshikv-fod: failed to create watch kthread, err=%ld\n",
			err);
		g_fod_watch_thread = NULL;
		return (int)err;
	}

	pr_info("hoshikv-fod: inbuilt FOD-HBM watch kthread created (disp=%d)\n",
		disp_id);
	return 0;
}

int mi_disp_lhbm_fod_watch_destroy(struct disp_feature *df, int disp_id)
{
	if (g_fod_watch_thread) {
		if (g_fod_watch_pressed && mi_get_disp_lhbm_fod(disp_id))
			mi_disp_lhbm_fod_set_disp_param(mi_get_disp_lhbm_fod(disp_id),
					LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
		g_fod_watch_pressed = false;
		atomic_set(&g_fod_watch_enable, 0);
		kthread_stop(g_fod_watch_thread);
		g_fod_watch_thread = NULL;
		pr_info("hoshikv-fod: inbuilt FOD-HBM watch kthread destroyed\n");
	}
	return 0;
}

int mi_disp_lhbm_fod_watch_enable(int disp_id, bool enable)
{
	atomic_set(&g_fod_watch_enable, enable ? 1 : 0);
	if (!enable && g_fod_watch_pressed) {
		/* hoshikv-fod v()-off: force LHBM OFF so the FOD light can never stay
		 * on after the service is disabled (previously LHBM got stuck because
		 * disabling the watch stopped the poller before it could see a 0, and
		 * fod_press_status stays '1' while the finger is held). We call the
		 * command sender directly, BYPASSING the local-hbm event path, so NO
		 * finger-up MI_DISP_EVENT_FOD is emitted -> the fod anim keeps running
		 * on v(), preserving the lib state machine (press=on, release=off,
		 * v() keeps anim). Reset pressed so a fresh edge can re-light on k().
		 */
		struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);
		cancel_delayed_work_sync(&g_fod_hbm_auto_off);
		if (lhbm_fod)
			mi_disp_lhbm_fod_set_disp_param(lhbm_fod,
					LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
		g_fod_watch_pressed = false;
		pr_info("hoshikv-fod: watch disabled, forced LHBM-OFF (no event)\n");
	}
	pr_info("hoshikv-fod: inbuilt FOD-HBM watch %s\n",
		enable ? "ENABLED" : "disabled");
	return 0;
}

