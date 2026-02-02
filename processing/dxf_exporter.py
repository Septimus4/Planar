"""Simple DXF exporter helpers for Planar (stub).
"""

try:
    import ezdxf
except Exception:
    ezdxf = None


def export_walls(lines, outpath):
    if ezdxf is None:
        with open(outpath, "w") as f:
            f.write("# ezdxf not installed; cannot write DXF\n")
        return
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    for (x1, y1, x2, y2) in lines:
        msp.add_line((x1, y1), (x2, y2), dxfattribs={"layer": "WALLS"})
    doc.saveas(outpath)
