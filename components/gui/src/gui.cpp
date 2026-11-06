#include "gui.hpp"

extern "C" {
#include "ui.h"
#include "ui_helpers.h"
// #include "ui_comp.h" // we don't have any components in this project yet
}

void Gui::deinit_ui() {
  logger_.info("Deinitializing UI");
  // delete the ui
  lv_obj_del(ui_MainScreen);
}

void Gui::init_ui() {
  logger_.info("Initializing UI");
  ui_init();

  set_usb_connected(false);
  set_ble_connected(false);

  // make the label and center it
  label_ = lv_label_create(lv_screen_active());
  lv_label_set_long_mode(label_, LV_LABEL_LONG_WRAP);
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_width(label_, 150);
}

void Gui::set_usb_connected(bool connected) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  // hide or show the usb image
  // ui_UsbIcon
  if (connected) {
    lv_obj_clear_flag(ui_UsbIcon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(ui_UsbIcon, LV_OBJ_FLAG_HIDDEN);
  }
}

void Gui::set_ble_connected(bool connected) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  // hide or show the ble image
  // ui_BtIcon
  if (connected) {
    lv_obj_clear_flag(ui_BtIcon, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(ui_BtIcon, LV_OBJ_FLAG_HIDDEN);
  }
}

void Gui::set_device_icon(const lv_image_dsc_t *icon) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  lv_image_set_src(ui_XboxIcon, icon);
  // the default icon is tinted white with an additive blend: draw this one as is
  lv_obj_set_style_image_recolor_opa(ui_XboxIcon, 0, 0 /* LV_PART_MAIN | LV_STATE_DEFAULT */);
  lv_obj_set_style_blend_mode(ui_XboxIcon, LV_BLEND_MODE_NORMAL,
                              0 /* LV_PART_MAIN | LV_STATE_DEFAULT */);
}

void Gui::set_host_icon(const lv_image_dsc_t *icon) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  lv_image_set_src(ui_SwitchIcon, icon);
}

void Gui::set_label_below_arrow(bool below) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  // widen the gap around the arrow so the label fits between the icon columns
  static constexpr int kColumnGap = 24;    // on each side of the 32 px arrow
  static constexpr int kLabelWidth = 80;   // arrow + both gaps
  static constexpr int kLabelOffsetY = 24; // just below the arrow
  if (below) {
    lv_obj_set_style_pad_column(ui_Panel1, kColumnGap, 0 /* LV_PART_MAIN | LV_STATE_DEFAULT */);
    // one line, truncated with an ellipsis, so a long name never covers the icons
    lv_label_set_long_mode(label_, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_size(label_, kLabelWidth,
                    lv_font_get_line_height(lv_obj_get_style_text_font(label_, LV_PART_MAIN)));
    lv_obj_align(label_, LV_ALIGN_CENTER, 0, kLabelOffsetY);
  } else {
    lv_obj_remove_local_style_prop(ui_Panel1, LV_STYLE_PAD_COLUMN,
                                   0 /* LV_PART_MAIN | LV_STATE_DEFAULT */);
    lv_label_set_long_mode(label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_size(label_, 150, LV_SIZE_CONTENT);
    lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  }
}

void Gui::on_value_changed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("Value changed: {}", fmt::ptr(target));
}

void Gui::on_pressed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("PRESSED: {}", fmt::ptr(target));
}

void Gui::on_scroll(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("SCROLL: {}", fmt::ptr(target));
}

void Gui::on_key(lv_event_t *e) {
  // print the key
  auto key = lv_indev_get_key(lv_indev_get_act());
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("KEY: {} on {}", key, fmt::ptr(target));
}
