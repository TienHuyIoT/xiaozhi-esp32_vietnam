#pragma once

#include <vector>
#include <string>

class LvglDisplay;

void StartImageDisplayTask(LvglDisplay* display,
                           std::vector<std::string> urls,
                           int duration_ms = 5000);

// Poll `server_url/api/image_queue` asynchronously and display the returned image.
// Called when the firmware detects a `% show_why_image` tool-call notification.
void StartImagePollingTask(LvglDisplay* display, const std::string& server_url);
