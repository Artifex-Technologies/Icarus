//! The Maps page's pictures of a table: a 2D graph (each row a line across the columns; a curve, one column, a line along
//! its rows) and a 3D surface (rows by columns, the value as height). Both are drawn with egui's painter, no 3D library:
//! a map of 16 x 16 is 225 flat quads, light enough for modest hardware. The 3D projection is orthographic: turned about
//! the vertical axis by `yaw`, then tilted by `pitch` (0 looks from the side, pi/2 from straight above), and painted
//! farthest first. Building the pictures is pure (`surface`, `lines`, tested); `draw_3d` and `draw_2d` only paint them.

use eframe::egui::{self, vec2, Color32, CornerRadius, Pos2, Rect, Sense, Shape, Stroke};

/// One cell of the surface, between four neighbouring values.
#[derive(Clone, Debug, PartialEq)]
pub struct Quad {
    /// On screen, in -RADIUS..RADIUS (x right, y down), going round the quad.
    pub corners: [[f32; 2]; 4],
    /// Larger is farther.
    pub depth: f32,
    /// 0..1: the mean height of its corners, for its colour.
    pub level: f32,
}

/// A value's point on screen, its depth, and its cell (row, column).
#[derive(Clone, Debug, PartialEq)]
pub struct Point {
    pub at: [f32; 2],
    pub depth: f32,
    pub row: usize,
    pub col: usize,
}

#[derive(Debug, Default)]
pub struct Surface {
    /// Farthest first, the order to paint them in.
    pub quads: Vec<Quad>,
    pub points: Vec<Point>,
}

/// Half the height of the surface (its width and depth go from -1 to 1).
const HEIGHT: f32 = 0.6;
/// The farthest any point of the surface can be from its middle, whichever way it is turned: the picture keeps its size.
pub const RADIUS: f32 = 1.54;

/// A point (x across, y away from the viewer, z up) turned by `yaw` and tilted by `pitch`: where it is on screen (y down)
/// and how far.
pub fn project(p: [f32; 3], yaw: f32, pitch: f32) -> ([f32; 2], f32) {
    let (s, c) = yaw.sin_cos();
    let x = p[0] * c - p[1] * s;
    let y = p[0] * s + p[1] * c;
    let (sp, cp) = pitch.sin_cos();
    let up = p[2] * cp + y * sp;
    let depth = y * cp - p[2] * sp;
    ([x, -up], depth)
}

/// The lowest and highest finite value.
fn range(z: &[Vec<f64>]) -> (f64, f64) {
    z.iter()
        .flatten()
        .filter(|v| v.is_finite())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
            (a.min(*v), b.max(*v))
        })
}

/// 0..1 for a value between `lo` and `hi`; the middle when all values are the same or the value is not a number.
fn level(v: f64, lo: f64, hi: f64) -> f32 {
    if hi > lo && v.is_finite() {
        ((v - lo) / (hi - lo)) as f32
    } else {
        0.5
    }
}

/// The surface of `z` (rows x columns; at least 2 x 2, otherwise there is none): row 0 at the back, as at the top of the
/// table, column 0 on the left.
pub fn surface(z: &[Vec<f64>], yaw: f32, pitch: f32) -> Surface {
    let rows = z.len();
    let cols = z.first().map_or(0, Vec::len);
    if rows < 2 || cols < 2 || z.iter().any(|r| r.len() != cols) {
        return Surface::default();
    }
    let (lo, hi) = range(z);
    let mut grid = vec![vec![([0.0f32; 2], 0.0f32); cols]; rows];
    let mut points = Vec::with_capacity(rows * cols);
    for (r, row) in z.iter().enumerate() {
        for (c, v) in row.iter().enumerate() {
            let x = -1.0 + 2.0 * c as f32 / (cols - 1) as f32;
            let y = 1.0 - 2.0 * r as f32 / (rows - 1) as f32;
            let h = (level(*v, lo, hi) - 0.5) * 2.0 * HEIGHT;
            let p = project([x, y, h], yaw, pitch);
            grid[r][c] = p;
            points.push(Point {
                at: p.0,
                depth: p.1,
                row: r,
                col: c,
            });
        }
    }
    let mut quads = Vec::with_capacity((rows - 1) * (cols - 1));
    for r in 0..rows - 1 {
        for c in 0..cols - 1 {
            let k = [(r, c), (r, c + 1), (r + 1, c + 1), (r + 1, c)];
            quads.push(Quad {
                corners: k.map(|(a, b)| grid[a][b].0),
                depth: k.iter().map(|(a, b)| grid[*a][*b].1).sum::<f32>() / 4.0,
                level: k.iter().map(|(a, b)| level(z[*a][*b], lo, hi)).sum::<f32>() / 4.0,
            });
        }
    }
    quads.sort_by(|a, b| b.depth.total_cmp(&a.depth));
    Surface { quads, points }
}

/// The 2D graph of `z`: one line per row across the columns, or for a single column one line along the rows. Points in
/// 0..1 (x right, y up); the i-th point of a line is cell (row, i) or, for one column, (i, 0).
pub fn lines(z: &[Vec<f64>]) -> Vec<Vec<[f32; 2]>> {
    let rows = z.len();
    let cols = z.first().map_or(0, Vec::len);
    let (lo, hi) = range(z);
    let x = |i: usize, n: usize| {
        if n > 1 {
            i as f32 / (n - 1) as f32
        } else {
            0.5
        }
    };
    if cols == 1 {
        return vec![(0..rows)
            .map(|r| [x(r, rows), level(z[r][0], lo, hi)])
            .collect()];
    }
    z.iter()
        .map(|row| {
            row.iter()
                .enumerate()
                .map(|(c, v)| [x(c, cols), level(*v, lo, hi)])
                .collect()
        })
        .collect()
}

/// The space a picture takes: the width left, 220 to 420 points high.
fn area(ui: &mut egui::Ui, sense: Sense) -> (Rect, egui::Response) {
    let h = ui.available_height().clamp(220.0, 420.0);
    ui.allocate_exact_size(vec2(ui.available_width(), h), sense)
}

/// Paints the surface of `z`; dragging turns it (left and right) and tilts it (up and down). Returns the cell whose point is
/// under the pointer, and whether it was clicked.
pub fn draw_3d(
    ui: &mut egui::Ui,
    z: &[Vec<f64>],
    yaw: &mut f32,
    pitch: &mut f32,
    background: Color32,
    line: Color32,
    color: impl Fn(f32) -> Color32,
) -> Option<((usize, usize), bool)> {
    let (rect, resp) = area(ui, Sense::click_and_drag());
    if resp.dragged() {
        let d = resp.drag_delta();
        *yaw -= d.x * 0.01;
        *pitch = (*pitch + d.y * 0.01).clamp(0.05, 1.55);
    }
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, CornerRadius::same(4), background);
    let s = surface(z, *yaw, *pitch);
    let scale = rect.width().min(rect.height()) / (2.0 * RADIUS);
    let to = |p: [f32; 2]| rect.center() + vec2(p[0], p[1]) * scale;
    let stroke = Stroke::new(1.0, line);
    for q in &s.quads {
        let p: Vec<Pos2> = q.corners.iter().map(|c| to(*c)).collect();
        let fill = color(q.level);
        // two triangles: a quad seen at a slant need not be convex
        painter.add(Shape::convex_polygon(
            vec![p[0], p[1], p[2]],
            fill,
            Stroke::NONE,
        ));
        painter.add(Shape::convex_polygon(
            vec![p[0], p[2], p[3]],
            fill,
            Stroke::NONE,
        ));
        painter.add(Shape::closed_line(p, stroke));
    }
    let hover = resp.hover_pos()?;
    let near = s
        .points
        .iter()
        .map(|p| (p, to(p.at).distance(hover)))
        .filter(|(_, d)| *d < 14.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))?
        .0;
    painter.circle_filled(to(near.at), 4.0, Color32::WHITE);
    Some(((near.row, near.col), resp.clicked()))
}

/// Paints the 2D graph of `z` (a line per row, coloured low to high by `color`). Returns the cell whose point is under the
/// pointer, and whether it was clicked.
pub fn draw_2d(
    ui: &mut egui::Ui,
    z: &[Vec<f64>],
    background: Color32,
    grid: Color32,
    color: impl Fn(f32) -> Color32,
) -> Option<((usize, usize), bool)> {
    let (rect, resp) = area(ui, Sense::click());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, CornerRadius::same(4), background);
    let inner = rect.shrink(14.0);
    for k in 0..=4 {
        let y = inner.bottom() - inner.height() * k as f32 / 4.0;
        painter.line_segment(
            [egui::pos2(inner.left(), y), egui::pos2(inner.right(), y)],
            Stroke::new(1.0, grid),
        );
    }
    let to = |p: [f32; 2]| {
        egui::pos2(
            inner.left() + p[0] * inner.width(),
            inner.bottom() - p[1] * inner.height(),
        )
    };
    let ls = lines(z);
    let one_column = z.first().is_some_and(|r| r.len() == 1);
    let mut near: Option<((usize, usize), f32, Pos2)> = None;
    for (i, l) in ls.iter().enumerate() {
        let f = if ls.len() > 1 {
            i as f32 / (ls.len() - 1) as f32
        } else {
            1.0
        };
        let pts: Vec<Pos2> = l.iter().map(|p| to(*p)).collect();
        painter.add(Shape::line(pts.clone(), Stroke::new(2.0, color(f))));
        if let Some(h) = resp.hover_pos() {
            for (j, p) in pts.iter().enumerate() {
                let d = p.distance(h);
                if d < 12.0 && near.is_none_or(|n| d < n.1) {
                    let cell = if one_column { (j, 0) } else { (i, j) };
                    near = Some((cell, d, *p));
                }
            }
        }
    }
    let (cell, _, at) = near?;
    painter.circle_filled(at, 4.0, Color32::WHITE);
    Some((cell, resp.clicked()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn close(a: [f32; 2], b: [f32; 2]) -> bool {
        (a[0] - b[0]).abs() < 1e-5 && (a[1] - b[1]).abs() < 1e-5
    }

    #[test]
    fn the_projection_turns_and_tilts() {
        // from straight above, unturned: x stays, farther is higher on screen, and higher is nearer
        let (p, d) = project([0.5, 1.0, 0.3], 0.0, FRAC_PI_2);
        assert!(close(p, [0.5, -1.0]), "{p:?}");
        assert!((d + 0.3).abs() < 1e-5);
        // from the side: height is up the screen, depth is distance
        let (p, d) = project([0.5, 1.0, 0.3], 0.0, 0.0);
        assert!(close(p, [0.5, -0.3]) && (d - 1.0).abs() < 1e-5);
        // a quarter turn takes the x axis to the y axis
        let (p, d) = project([1.0, 0.0, 0.0], FRAC_PI_2, 0.0);
        assert!(close(p, [0.0, 0.0]) && (d - 1.0).abs() < 1e-5, "{p:?} {d}");
        // nothing leaves the circle the picture is fitted to, however it is turned
        for yaw in [0.0f32, 0.7, 2.0, 4.0] {
            for pitch in [0.05f32, 0.6, 1.55] {
                for c in [
                    [1.0, 1.0, HEIGHT],
                    [-1.0, 1.0, -HEIGHT],
                    [1.0, -1.0, HEIGHT],
                ] {
                    let (p, _) = project(c, yaw, pitch);
                    assert!(p[0].hypot(p[1]) <= RADIUS);
                }
            }
        }
    }

    #[test]
    fn a_map_becomes_quads_painted_farthest_first() {
        let z = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 9.0]];
        let s = surface(&z, 0.4, 0.8);
        assert_eq!(s.quads.len(), 2, "(rows - 1) x (columns - 1)");
        assert_eq!(s.points.len(), 6);
        assert!(s.quads.windows(2).all(|w| w[0].depth >= w[1].depth));
        // the right-hand quad holds the 9, the top of the map
        let right = s
            .quads
            .iter()
            .max_by(|a, b| a.level.total_cmp(&b.level))
            .unwrap();
        assert!((right.level - (2.0 + 3.0 + 9.0 + 5.0 - 4.0 * 1.0) / 4.0 / 8.0).abs() < 1e-5);
        // seen from above, unturned, row 0 is at the top and column 0 on the left
        let top = surface(&z, 0.0, FRAC_PI_2);
        let at = |r: usize, c: usize| {
            top.points
                .iter()
                .find(|p| p.row == r && p.col == c)
                .unwrap()
                .at
        };
        assert!(at(0, 0)[1] < at(1, 0)[1] && at(0, 0)[0] < at(0, 2)[0]);
        // the same values everywhere: flat, in the middle, not a division by zero
        let flat = surface(&[vec![7.0; 3], vec![7.0; 3]], 0.3, 0.9);
        assert!(flat.quads.iter().all(|q| q.level == 0.5));
        // a curve or a single value has no surface
        assert!(surface(&[vec![1.0], vec![2.0]], 0.0, 1.0).quads.is_empty());
        assert!(surface(&[vec![1.0, 2.0]], 0.0, 1.0).quads.is_empty());
    }

    #[test]
    fn the_2d_graph_has_a_line_per_row_or_one_along_a_curve() {
        let map = lines(&[vec![0.0, 5.0, 10.0], vec![10.0, 5.0, 0.0]]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[0], vec![[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]]);
        let curve = lines(&[vec![2.0], vec![4.0], vec![3.0]]);
        assert_eq!(curve, vec![vec![[0.0, 0.0], [0.5, 1.0], [1.0, 0.5]]]);
        assert_eq!(
            lines(&[vec![f64::NAN, 1.0]])[0][0][1],
            0.5,
            "not a number: drawn in the middle"
        );
    }
}
