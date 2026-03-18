# Generates ONE hex file per macroblock — no frame array indexing needed
import os

YUV  = "frame.yuv"
W, H = 320, 176

raw = open(YUV, 'rb').read()
Y_size  = W * H
UV_size = (W//2) * (H//2)

planes = {
    'y': (0,               W,    H,    32, 32),
    'u': (Y_size,          W//2, H//2, 16, 16),
    'v': (Y_size+UV_size,  W//2, H//2, 16, 16),
}

for plane, (offset, fw, fh, mbw, mbh) in planes.items():
    cols = fw // mbw
    rows = fh // mbh
    blk  = 0
    for r in range(rows):
        for c in range(cols):
            fname = f"block_{plane}_{blk:02d}.hex"
            with open(fname, 'w') as f:
                for py in range(mbh):
                    for px in range(mbw):
                        fy  = r*mbh + py
                        fx  = c*mbw + px
                        idx = offset + fy*fw + fx
                        val = raw[idx] if idx < len(raw) else 128
                        f.write(f"{val:02x}\n")
            blk += 1
    print(f"{plane.upper()}: {blk} files written (block_{plane}_00.hex .. block_{plane}_{blk-1:02d}.hex)")
