"""Hulpfuncties voor 3S li-ionbatterijen (18650/21700)."""

# Spanning per cel (V) -> resterende lading (%) voor li-ion in rust. Benadering: onder
# belasting lijkt de batterij leger, tijdens het laden voller dan ze is.
CELL_CURVE = [
    (3.30, 0), (3.50, 5), (3.60, 10), (3.70, 25), (3.80, 45),
    (3.90, 62), (4.00, 77), (4.10, 90), (4.20, 100),
]


def percentage(voltage, cells=3):
    """Geschatte lading (0-100) uit de pakketspanning."""
    v = voltage / cells
    if v <= CELL_CURVE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(CELL_CURVE, CELL_CURVE[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0
