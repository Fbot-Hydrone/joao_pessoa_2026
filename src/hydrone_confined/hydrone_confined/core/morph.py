"""Binary morphology and labelling on small bool grids, numpy only (no scipy)."""
import numpy as np


def disc_offsets(radius_cells):
    """(di, dj) offsets inside a disc of radius_cells (float ok)."""
    r = int(np.floor(radius_cells))
    out = []
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj <= radius_cells * radius_cells + 1e-9:
                out.append((di, dj))
    return out


def dilate(mask, radius_cells):
    """Dilate by a disc. Edges don't wrap."""
    if radius_cells <= 0:
        return mask.copy()
    r = int(np.floor(radius_cells))
    h, w = mask.shape
    big = np.zeros((h + 2 * r, w + 2 * r), dtype=bool)
    for di, dj in disc_offsets(radius_cells):
        big[r + di:r + di + h, r + dj:r + dj + w] |= mask
    return big[r:r + h, r:r + w]


def erode(mask, radius_cells):
    """Erode by a disc; outside the grid counts as set, so edges don't eat in."""
    return ~dilate(~mask, radius_cells)


def close(mask, radius_cells):
    """Closing that also works at the grid edge (padded first)."""
    r = int(np.ceil(radius_cells)) + 1
    big = np.pad(mask, r)
    return erode(dilate(big, radius_cells), radius_cells)[r:-r, r:-r]


def open_(mask, radius_cells):
    return dilate(erode(mask, radius_cells), radius_cells)


def dilate4(mask):
    """One 4-connected step."""
    out = mask.copy()
    out[1:] |= mask[:-1]
    out[:-1] |= mask[1:]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def dilate8(mask):
    out = dilate4(mask)
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def flood(seed, allowed, conn=4):
    """Everything in `allowed` connected to `seed`."""
    grow = dilate4 if conn == 4 else dilate8
    cur = seed & allowed
    while True:
        nxt = grow(cur) & allowed
        if (nxt == cur).all():
            return cur
        cur = nxt


def fill_holes(mask):
    """Set every unset region not connected to the border."""
    border = np.zeros_like(mask)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    outside = flood(border & ~mask, ~mask)
    return ~outside


def label(mask, conn=8):
    """Connected components. Returns (labels int32, n); 0 is background."""
    lab = np.zeros(mask.shape, dtype=np.int32)
    h, w = mask.shape
    if conn == 8:
        nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    else:
        nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1))
    n = 0
    for i0, j0 in np.argwhere(mask):
        if lab[i0, j0]:
            continue
        n += 1
        lab[i0, j0] = n
        stack = [(i0, j0)]
        while stack:
            i, j = stack.pop()
            for di, dj in nbrs:
                a, b = i + di, j + dj
                if 0 <= a < h and 0 <= b < w and mask[a, b] and not lab[a, b]:
                    lab[a, b] = n
                    stack.append((a, b))
    return lab, n
