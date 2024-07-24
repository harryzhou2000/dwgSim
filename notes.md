# DwgSim Notes

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [DwgSim Notes](#dwgsim-notes)
  - [Examination Process](#examination-process)
  - [Structure of Output](#structure-of-output)
  - [Entity Conversion](#entity-conversion)
    - [LINE](#line)
    - [ARC](#arc)
    - [CIRCLE](#circle)
    - [ELLIPSE](#ellipse)
    - [POLYLINE](#polyline)
    - [LWPOLYLINE](#lwpolyline)
    - [SPLINE](#spline)
      - [Cubic Spline to B-Spline Conversion](#cubic-spline-to-b-spline-conversion)
    - [INSERT](#insert)

<!-- /code_chunk_output -->

## Examination Process

Using AutoCAD 2016 for producing .dwg and xxx-ac.dxf (R2013) example files.

The xxx-ac.dxf files are used as reference, and .dwg files are input of dwgSim and converted to .json and .dxf files.

Using libreCAD to try opening converted dxf files.

## Structure of Output

The output DXF or JSON consists of geometric entities.

The ENTITY section of DXF is basically the "model space" block, which is the `block_header` got by `dwg_model_space_ref`, consists of a list of geometric entities. 

The BLOCK section of DXF now consists of multiple user-defined block entries, as in `block_control->entries[i]`. The BLOCK objects each contains its name and handle, and a list of entities just like model space.

The blocks are referenced by INSERT entities.

## Entity Conversion

Currently, we only handle very basic 1-D shapes. JSON segments are used to demonstrate the data fields of these entities.

### LINE

```json
    {
      "type": "LINE",
      "layerId": 116,
      "start": [
        -3.3467508846605478e-14,4.999999999999208,0.0
      ],
      "end": [
        415.00000000000008,5.000000000000172,0.0
      ],
      "extrusion": [
        0.0,0.0,1.0
      ]
    }
```

### ARC

```json
    {
      "type": "ARC",
      "layerId": 16,
      "center": [
        637.5292347860208,163.846720972691,0.0
      ],
      "radius": 0.25,
      "start_angle": 4.712388980384704,
      "end_angle": 0.0,
      "extrusion": [
        0.0,0.0,1.0
      ]
    }
```

Note that JSON stores `start_angle` and `end_angle` as radian values, while in DXF it needs to be converted to degrees.

### CIRCLE

```json
   {
      "type": "CIRCLE",
      "layerId": 16,
      "center": [
        541.7792347860208,
        174.096720972691,
        0.0
      ],
      "radius": 2.7499999999999837,
      "extrusion": [
        0.0,
        0.0,
        1.0
      ]
    }
```

### ELLIPSE

```json
    {
      "type": "ELLIPSE",
      "handle": 740,
      "layerId": 16,
      "center": [
        974.3565815644028,2248.436600743432,0.0
      ],
      "sm_axis": [
        0.0,-997.2928890305439,0.0
      ],
      "axis_ratio": 0.495759305309735,
      "start_angle": 1.8011948972371866,
      "end_angle": 3.9356000903280585,
      "extrusion": [
        0.0,0.0,0.9999999999999998
      ]
    }
```

`sm_axis` is the main semi axis end point's coordinate relative to the center.

Note that in JSON `start_angle` and `end_angle` are in radian. **Not sure if they should be in degrees in DXF.**

Note that according to documentation, the `sm_axis` and `center` are in WCS.

### POLYLINE

In JSON, polylines of 2D and 3D type are separately stored in `POLYLINE_2D` and `POLYLINE_3D` entities.

```json
    {
      "type": "POLYLINE_2D",
      "handle": 37522,
      "layerId": 2262,
      "flag": 0,
      "vertex": [
        [
          148.53390349724499,584.179790930212,0.0
        ],
        [
          141.03390349724499,584.179790930212,0.0
        ],
        [
          141.03390349724499,589.054790930212,0.0
        ],
        [
          141.03390349724499,584.179790930212,0.0
        ]
      ],
      "bulge": [
        0.0,
        0.0,
        0.0,
        0.0
      ],
      "vertexHandles": [
        37524,
        37525,
        37526,
        37527
      ],
      "seqendHandle": 37523,
      "extrusion": [
        0.0,0.0,1.0
      ]
    }
```

and `POLYLINE_3D`'s JSON is almost the same as 2D. The VERTEX child entities are compressed into the arrays, but their handles are preserved for DXF output.

### LWPOLYLINE

```json
    {
      "type": "LWPOLYLINE",
      "handle": 31478,
      "layerId": 32167,
      "flag": 256,
      "vertex": [
        [
          108.78390349724498,624.1051206612086
        ],
        [
          108.37742222784743,623.6956094372051
        ],
        [
          108.28390349724498,623.6013583467043
        ]
      ],
      "bulge": [],
      "extrusion": [
        0.0,0.0,0.0
      ]
    }
```

`LWPOLYLINE` seems to be an older type and newer AutoCAD only creates `POLYLINE_2D`. However, in the DWG created with SolidWorks output, this entity exists.

### SPLINE

```json
    {
      "type": "SPLINE",
      "handle": 688,
      "layerId": 16,
      "flag": 1064,
      "splineflags": 9,
      "periodic": 0,
      "rational": 0,
      "weighted": 0,
      "knotparam": 0,
      "ctrl_tol": 1e-7,
      "fit_tol": 1e-10,
      "knot_tol": 1e-7,
      "degree": 3,
      "beg_tan_vec": [
        0.0,0.0,0.0
      ],
      "end_tan_vec": [
        0.0,0.0,0.0
      ],
      "ctrl_pts": [],
      "fit_pts": [
        [
          1,2,0
        ],
        [
          2,1,0
        ],
        [
          1,1,0
        ]
      ],
      "knots": []
    }
```

The AutoCAD spline is generally a NURBS curve, but it has two operating modes. The info below is inferred from the actual DXF output and not seen in documents.

In the mode above where `ctrl_pts` is empty and `fit_pts` is not empty, the operating mode is fitting. The fitting mode used **cubic spline** as interpolation method.
$$
\mathbf{x}(t)=\sum_{i=1}^{N_f} S_i(t) \mathbf{f}_i
$$
where $\mathbf{f}_i$ are fitting points and $S_i(t)$ are spline interpolation bases. The bases are piecewise cubic polynomials in parameter space of $t$. The notation above uses a Lagrangian type basis, but in real life we would like to use cubic Hermit-interpolation basis or cubic Bézier basis. Under the framework of NURBS, we would use cubic Bézier basis:
$$
\mathbf{x}(t)=\sum_{i=1}^{N_b} B_i(t) \mathbf{c}_i
$$
where control points $\mathbf{c}_i$ are solved using the interpolation conditions and start/end conditions.

When `fit_pts` is empty and `ctrl_pts` is present, the spline is created with purely control points as a B-spline.
Normally, by default this B-spline is cubic, using knots of $[0,0,0,0,1,...,N-3,N-3, N-3, N-3]$.

LibreCAD, however, can't read a DXF with only `fit_pts`. LibreCAD tries to treat the fit points as control points, which result in, of course, a faulty curve to be imported. The start and end points coincide with the correct curve, but the body of curve is different.

When AutoCAD exports a DXF, the cubic spline is converted to a B-spline, which makes the exported DXF to have both `ctrl_pts` and `fit_pts`. Sadly, when reading a DWG with libredwg, the `fit_pts` only situation requires the DWG reader to do the conversion manually. libredwg's dwg2dxf program fails to do the conversion and therefore the DXF spline re-imported with LibreCAD becomes wrong.

#### Cubic Spline to B-Spline Conversion

A cubic spline can be expressed as a B-spline precisely, as a cubic B-Spline can represent the completely represent the piecewise polynomial space. Moreover, when the internal knots of the B-spline are non-duplicate, the bases of the B-spline naturally satisfy the continuous condition used to define cubic spline. Therefore, the cubic-spline with fitting point knots:
$$
[t_0, t_1, t_2, t_3,...,t_N]
$$
could be expressed with cubic B-Spline using $N+6$ knots:
$$
[t_0, t_0, t_0, t_0, t_1, t_2, t_3,...,t_N, t_N, t_N, t_N]
$$
and $N+2$ control points $\mathbf{c}_1,...\mathbf{c}_{N+2}$. Combined with boundary conditions, the conversion is basically a linear solving procedure.

Recall
$$
\mathbf{x}(t)=\sum_{i=1}^{N_b = N+2} B_i(t) \mathbf{c}_i
$$
in order to obtain the control points $\mathbf{c}_i$, we want fit point conditions:
$$
\mathbf{x}(t_i) = \mathbf{f}_i, i=1,2...N_f
$$
and also boundary conditions (for open splines):
$$
\frac{\mathrm{d}\,\mathbf{x}(t)}{\mathrm{d}\, t}|_{t=t_0} = \mathbf{T}_0,\ \ \ \ 
\frac{\mathrm{d}\,\mathbf{x}(t)}{\mathrm{d}\, t}|_{t=t_{N}} = \mathbf{T}_{end}
$$
where tangential $\mathbf{T}$ vectors are specified with `beg_tan_vec` and `end_tan_vec`.
If either of them is zero, the corresponding boundary condition is replaced with a natural condition:
$$
\frac{\mathrm{d}^2\,\mathbf{x}(t)}{\mathrm{d}\, t^2}|_{t=t_0} = 0,\ \ \ \ \text{or}\ \ 
\frac{\mathrm{d}^2\,\mathbf{x}(t)}{\mathrm{d}\, t^2}|_{t=t_{N}} = 0
$$
If the `splineflags`'s 3rd bit indicates the fitting line is closed (```splineflags & 0x04 == true``` ), periodic BC must be used instead:
$$
\begin{aligned}
\frac{\mathrm{d}\,\mathbf{x}(t)}{\mathrm{d}\, t}|_{t=t_0} & =
\frac{\mathrm{d}\,\mathbf{x}(t)}{\mathrm{d}\, t}|_{t=t_{N}} \\
\frac{\mathrm{d}^2\,\mathbf{x}(t)}{\mathrm{d}\, t^2}|_{t=t_0} & =
\frac{\mathrm{d}^2\,\mathbf{x}(t)}{\mathrm{d}\, t^2}|_{t=t_{N}} \\
\end{aligned}
$$

The corresponding solving procedure is implemented in the `CubicSplineToBSpline` function in `splineUtil.h`, which is basically a dense linear solving.

The testing program `testSplineConversion` is used to test the conversion results against the AutoCAD's output. For 1.open lines with both ends natural boundary condition, 2.both ends given tangential vectors, 3.one end given tangential vectors and 4.closed fitting lines, the conversion almost exactly matches the output of AutoCAD's DXF export.

### INSERT

```json
    {
      "type": "INSERT",
      "handle": 621,
      "layerId": 16,
      "blockId": 615,
      "blockName": "block1",
      "ins_pt": [
        1386.6189082221736,607.096574752356,0.0
      ],
      "scale": [
        1.0,1.0,1.0
      ],
      "rotation": 0.0,
      "num_cols": 1,
      "num_rows": 1,
      "col_spacing": 0.0,
      "row_spacing": 0.0,
      "extrusion": [
        0.0,0.0,1.0
      ]
    }
```

The `blockId` and `blockName` both refers to the unique block. `layerId` is converted to the corresponding layer's name in the DXF output. 

The `rotation` field is in radian, and must be converted to degrees when output in the DXF.

Somehow, libredwg gives INSERT's `num_cols` and `num_rows` as 0 by default, but this is erroneous when put into DXF. So the JSON document records `max(1,num_cols)` (and rows).
**It is not sure if this is appropriate in all situations.**
<!-- ! WARNING -->

