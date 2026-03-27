# ST7789 Display Notes

## Hardware

- Panel: 1.9" IPS ST7789, 320×170px, landscape
- Interface: SPI (CLK=18, MOSI=17, CS=15, DC=16, RST=21, BL=38)
- Driver stack: `esp_lcd` + `esp_lvgl_port` v2.3+ + LVGL 9

---

## Key Implementation Findings

### 1. Rotation must be applied AFTER `lvgl_port_add_disp()`

`esp_lvgl_port` resets the ST7789 MADCTL register internally during
`lvgl_port_add_disp()`, wiping any rotation previously set. Always apply
`esp_lcd_panel_swap_xy()`, `esp_lcd_panel_mirror()`, and
`esp_lcd_panel_set_gap()` **after** that call.

```c
s_lvgl_disp = lvgl_port_add_disp(&disp_cfg);

// Rotation AFTER lvgl_port_add_disp — not before
ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 35));
```

### 2. LVGL port task must be pinned to Core 0

The default `task_affinity = -1` allows the LVGL task to migrate to Core 1,
where it directly interferes with the audio pipeline (priority 7). This causes
progressive audio buffer backpressure — latency climbs from ~1800ms to 10000ms
without recovery, eventually causing stream misalignment and RTSP crypto errors.

Fix: explicitly set `task_affinity = 0` in the port config struct. Do **not**
use the `ESP_LVGL_PORT_INIT_CONFIG()` macro as it defaults to -1.

```c
const lvgl_port_cfg_t lvgl_cfg = {
    .task_priority     = 4,
    .task_stack        = 6144,
    .task_affinity     = 0,   // Core 0 — keep Core 1 free for audio
    .task_max_sleep_ms = 500,
    .timer_period_ms   = 5,
};
```

Rule: all display tasks on Core 0, all audio tasks on Core 1.

---

## Background Image

### Colour Depth Limitation

The ST7789 in this configuration runs **RGB565** (16-bit colour):

| Channel | Source depth | Display depth |
|---------|-------------|---------------|
| Red     | 8 bits (256 levels) | 5 bits (32 levels) |
| Green   | 8 bits (256 levels) | 6 bits (64 levels) |
| Blue    | 8 bits (256 levels) | 5 bits (32 levels) |

This causes **visible banding** on subtle dark gradients. It is a fundamental
hardware limitation and cannot be fixed in software. Design backgrounds with
this in mind — bold contrast and distinct colour regions work better than
smooth dark-to-dark transitions.

### Converting a PNG to a C Array

Use this Python script. Confirmed working byte order (little-endian RGB565)
was determined via a colour bar test pattern.

```python
from PIL import Image, ImageEnhance
import os

img = Image.open('your_background.png').convert('RGB').resize((320, 170))

# Reduce brightness to compensate for ST7789 backlight (adjust 0.4–0.6 to taste)
img = ImageEnhance.Brightness(img).enhance(0.5)

# Optional: Floyd-Steinberg dithering reduces banding on gradients
img = img.convert('P', palette=Image.ADAPTIVE,
                  dither=Image.FLOYDSTEINBERG, colors=256).convert('RGB')

W, H = 320, 170
pixels = img.load()
bytes_out = []
for y in range(H):
    for x in range(W):
        r, g, b = pixels[x, y]
        rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        bytes_out.append(rgb565 & 0xFF)          # little-endian
        bytes_out.append((rgb565 >> 8) & 0xFF)

# Write background.c
# ... (see components/display/background.h for full file format)
```

### Swapping the Background Image

1. Export new image as exactly **320×170px PNG**
2. Run the script above, adjusting brightness to taste
3. Replace `components/display/background.c` — the array must be named
   `background_map[]` and the descriptor `background_img`
4. Rebuild — no other files need to change

### Why Not the LVGL Online Converter?

The LVGL v9 online converter (lvgl.io/tools/imageconverter) does not expose
a byte-swap option for RGB565. The Python script above gives full control and
produces the correct little-endian output for this display configuration.

---

## Diagnostic: Colour Bar Test

If the background image colours look wrong (washed out, swapped channels,
or psychedelic), generate a colour bar test pattern to confirm byte order:

```python
from PIL import Image, ImageDraw

bars = [
    (255,255,255), (255,255,0), (0,255,255), (0,255,0),
    (255,0,255),   (255,0,0),   (0,0,255),
]
img = Image.new('RGB', (320, 170))
draw = ImageDraw.Draw(img)
bar_w = 320 // len(bars)
for i, color in enumerate(bars):
    draw.rectangle([i*bar_w, 0, i*bar_w+bar_w, 170], fill=color)
```

Expected result on screen (left to right):
**White → Yellow → Cyan → Green → Magenta → Red → Blue**

If the order matches, byte order is correct. If channels are swapped,
check whether `swap_bytes` in `lvgl_port_display_cfg_t` matches the
byte order of the image data.
