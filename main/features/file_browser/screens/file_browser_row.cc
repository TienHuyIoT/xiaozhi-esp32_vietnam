#include "file_browser_row.h"

#include <stdio.h>
#include <time.h>

namespace {
constexpr lv_color_t kTextPrimary = LV_COLOR_MAKE(0xEE, 0xEE, 0xEE);
constexpr lv_color_t kTextSecondary = LV_COLOR_MAKE(0xAA, 0xAA, 0xAA);
constexpr lv_color_t kBgFocused = LV_COLOR_MAKE(0x22, 0x22, 0x22);
constexpr lv_color_t kBorderFocused = LV_COLOR_MAKE(0x00, 0xCC, 0xFF);
}

namespace {
class RowRenderer final {
 public:
  static RowRenderer& Instance() {
    static RowRenderer instance;
    return instance;
  }

  lv_obj_t* Create(lv_obj_t* parent, const file_item_t* item) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_size(row, 320, 32);
    lv_obj_set_style_pad_all(row, 4, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_set_user_data(row, const_cast<file_item_t*>(item));

    lv_obj_t* icon = lv_label_create(row);
    lv_label_set_text(icon, (item != nullptr && item->type == FILE_TYPE_FOLDER) ? "[D]" : "[F]");
    lv_obj_set_width(icon, 30);
    lv_obj_set_style_text_color(icon, kTextPrimary, 0);

    lv_obj_t* name = lv_label_create(row);
    lv_obj_set_width(name, 150);
    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
    lv_label_set_text(name, (item != nullptr) ? item->name : "");
    lv_obj_set_style_text_color(name, kTextPrimary, 0);

    lv_obj_t* size = lv_label_create(row);
    lv_obj_set_width(size, 56);
    lv_obj_set_style_text_align(size, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(size, kTextSecondary, 0);
    char size_buf[20];
    if (item != nullptr && item->type != FILE_TYPE_FOLDER) {
      file_browser_format_size(item->size_bytes, size_buf, sizeof(size_buf));
    } else {
      snprintf(size_buf, sizeof(size_buf), "--");
    }
    lv_label_set_text(size, size_buf);

    lv_obj_t* date = lv_label_create(row);
    lv_obj_set_width(date, 80);
    lv_obj_set_style_text_align(date, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(date, kTextSecondary, 0);
    char date_buf[20];
    file_browser_format_date((item != nullptr) ? item->date_unix : 0, date_buf,
                             sizeof(date_buf));
    lv_label_set_text(date, date_buf);

    return row;
  }

  void SetFocus(lv_obj_t* row, bool focused) {
    if (row == nullptr) {
      return;
    }

    if (focused) {
      lv_obj_set_style_bg_color(row, kBgFocused, 0);
      lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
      lv_obj_set_style_border_side(row, LV_BORDER_SIDE_LEFT, 0);
      lv_obj_set_style_border_width(row, 4, 0);
      lv_obj_set_style_border_color(row, kBorderFocused, 0);
    } else {
      lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
      lv_obj_set_style_border_width(row, 0, 0);
    }
  }

  void SetSelected(lv_obj_t* row, bool selected) {
    if (row == nullptr) {
      return;
    }
    if (selected) {
      lv_obj_set_style_bg_opa(row, LV_OPA_50, 0);
    }
  }

  void SetDisabled(lv_obj_t* row, bool disabled) {
    if (row == nullptr) {
      return;
    }
    if (disabled) {
      lv_obj_add_state(row, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(row, LV_STATE_DISABLED);
    }
  }

  const file_item_t* GetItem(lv_obj_t* row) const {
    if (row == nullptr) {
      return nullptr;
    }
    return static_cast<const file_item_t*>(lv_obj_get_user_data(row));
  }

  void FormatSize(uint32_t size_bytes, char* buf, size_t buf_size) const {
    if (buf == nullptr || buf_size == 0) {
      return;
    }

    if (size_bytes < 1024U) {
      snprintf(buf, buf_size, "%u B", static_cast<unsigned>(size_bytes));
      return;
    }

    double size = static_cast<double>(size_bytes) / 1024.0;
    if (size < 1024.0) {
      snprintf(buf, buf_size, (size < 10.0) ? "%.1f KiB" : "%.0f KiB", size);
      return;
    }

    size /= 1024.0;
    if (size < 1024.0) {
      snprintf(buf, buf_size, (size < 10.0) ? "%.1f MiB" : "%.0f MiB", size);
      return;
    }

    size /= 1024.0;
    snprintf(buf, buf_size, "%.2f GiB", size);
  }

  void FormatDate(uint32_t unix_time, char* buf, size_t buf_size) const {
    if (buf == nullptr || buf_size == 0) {
      return;
    }
    if (unix_time == 0) {
      snprintf(buf, buf_size, "---- -- --:--");
      return;
    }

    time_t t = static_cast<time_t>(unix_time);
    struct tm time_info;
#if defined(_WIN32)
    localtime_s(&time_info, &t);
#else
    localtime_r(&t, &time_info);
#endif
    strftime(buf, buf_size, "%Y-%m-%d %H:%M", &time_info);
  }

  uint16_t IconId(file_type_t type, bool is_accessible) const {
    if (!is_accessible) {
      return static_cast<uint16_t>(FILE_TYPE_UNSUPPORTED);
    }
    return static_cast<uint16_t>(type);
  }
};
}  // namespace

extern "C" lv_obj_t* file_browser_row_create(lv_obj_t* parent,
                                             const file_item_t* item) {
  return RowRenderer::Instance().Create(parent, item);
}

extern "C" void file_browser_row_set_focus(lv_obj_t* row, bool focused) {
  RowRenderer::Instance().SetFocus(row, focused);
}

extern "C" void file_browser_row_set_selected(lv_obj_t* row, bool selected) {
  RowRenderer::Instance().SetSelected(row, selected);
}

extern "C" void file_browser_row_set_disabled(lv_obj_t* row, bool disabled) {
  RowRenderer::Instance().SetDisabled(row, disabled);
}

extern "C" const file_item_t* file_browser_row_get_item(lv_obj_t* row) {
  return RowRenderer::Instance().GetItem(row);
}

extern "C" void file_browser_format_size(uint32_t size_bytes, char* buf, size_t buf_size) {
  RowRenderer::Instance().FormatSize(size_bytes, buf, buf_size);
}

extern "C" void file_browser_format_date(uint32_t unix_time, char* buf, size_t buf_size) {
  RowRenderer::Instance().FormatDate(unix_time, buf, buf_size);
}

extern "C" uint16_t file_browser_get_icon_id(file_type_t type, bool is_accessible) {
  return RowRenderer::Instance().IconId(type, is_accessible);
}