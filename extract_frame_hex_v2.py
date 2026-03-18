"""
extract_frame_hex_v2.py
──────────────────────────────────────────────────────────────────────────────
FIXES vs v1 (extract_frame_hex.py):

  BUG FIXED — BOTH Y and UV were dropping pixels at the bottom of the frame:
    320×176:
      Y  176/32 = 5.5 → v1 only extracted 5 block-rows (160px), dropped 16 rows
                          = 5,120 Y pixels LOST per frame
      UV  88/16 = 5.5 → v1 only extracted 5 block-rows (80px),  dropped 8 rows

    5MP 2592×1944:
      Y  1944/32 = 60.75 → v1 dropped 24 rows = 62,208 Y pixels
      UV  972/16 = 60.75 → v1 dropped 12 rows

  FIX: use math.ceil() for block-row count. Partial blocks at the frame edge
  are padded with the last valid row's pixel value (edge-extend), which gives
  better compression than padding with 0 or 128.

OUTPUTS per plane:
  - Individual block hex files:  block_y_00.hex .. block_y_NN.hex
  - Combined hex file:           frame_y_all.hex   (blocks concatenated)
  - These are used by testbenches and the Verilog encoder pipeline.

BLOCK COUNTS (correct after this fix):
  320×176  Y:  ceil(176/32) × (320/32) = 6×10 = 60 blocks  (was 50)
  320×176  UV: ceil( 88/16) × (160/16) = 6×10 = 60 blocks  (was 50)
  5MP 2592×1944  Y:  61×81 = 4,941  UV: 61×81 = 4,941
──────────────────────────────────────────────────────────────────────────────
"""
import os
import math

YUV  = "frame.yuv"
W, H = 320, 176

raw = open(YUV, 'rb').read()
Y_size  = W * H
UV_size = (W // 2) * (H // 2)

planes = {
    #  name   offset       fw      fh      mbw  mbh
    'y': (0,            W,      H,      32,  32),
    'u': (Y_size,       W//2,   H//2,   16,  16),
    'v': (Y_size+UV_size, W//2, H//2,   16,  16),
}

for plane, (offset, fw, fh, mbw, mbh) in planes.items():
    cols = fw // mbw                  # always exact (frame width divisible)
    rows = math.ceil(fh / mbh)        # FIX: ceil so partial bottom row is included

    all_pixels = []
    blk = 0
    for r in range(rows):
        for c in range(cols):
            fname = f"block_{plane}_{blk:02d}.hex"
            with open(fname, 'w') as f:
                for py in range(mbh):
                    for px in range(mbw):
                        fy = r * mbh + py
                        fx = c * mbw + px
                        # FIX: edge-extend instead of padding with 128
                        # clamp fy to last valid row so bottom padding uses
                        # actual edge pixels (better for compression)
                        fy_clamped = min(fy, fh - 1)
                        fx_clamped = min(fx, fw - 1)
                        idx = offset + fy_clamped * fw + fx_clamped
                        val = raw[idx] if idx < len(raw) else 128
                        f.write(f"{val:02x}\n")
                        all_pixels.append(val)
            blk += 1

    # Write combined hex for testbench ($readmemh)
    combined_fname = f"frame_{plane}_all.hex"
    with open(combined_fname, 'w') as f:
        for v in all_pixels:
            f.write(f"{v:02x}\n")

    print(f"{plane.upper()}: {blk} blocks, {blk * mbw * mbh} pixels extracted")
    print(f"      block_{plane}_00.hex .. block_{plane}_{blk-1:02d}.hex")
    print(f"      {combined_fname}  ({len(all_pixels)} bytes)")
    if fh % mbh != 0:
        pad_rows = mbh - (fh % mbh)
        print(f"      NOTE: {pad_rows} padding rows added (edge-extended, not zeros)")
    print()

print("Done. Update testbenches to use 60 Y blocks and 60 UV blocks for 320x176.")
print("For 5MP 2592x1944: 4941 Y blocks, 4941 UV blocks each.")
