package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.Point;

import java.util.ArrayList;
import java.util.List;

public class GridLineTraversalLine {
    public List<Point<Integer>> points = new ArrayList<Point<Integer>>();

    public static void gridLineCore(Point<Integer> start, Point<Integer> end, GridLineTraversalLine line) {
        int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
        int cnt = 0;

        dx = Math.abs(end.x - start.x);
        dy = Math.abs(end.y - start.y);

        if (dy <= dx) {
            d = 2 * dy - dx;
            incr1 = 2 * dy;
            incr2 = 2 * (dy - dx);
            if (start.x > end.x) {
                x = end.x;
                y = end.y;
                ydirflag = (-1);
                xend = start.x;
            } else {
                x = start.x;
                y = start.y;
                ydirflag = 1;
                xend = end.x;
            }
            line.points.add(cnt, new Point<Integer>(x, y));
            cnt++;
            if (((end.y - start.y) * ydirflag) > 0) {
                while (x < xend) {
                    x++;
                    if (d < 0) {
                        d += incr1;
                    } else {
                        y++;
                        d += incr2;
                    }
                    line.points.add(cnt, new Point<Integer>(x, y));
                    cnt++;
                }
            } else {
                while (x < xend) {
                    x++;
                    if (d < 0) {
                        d += incr1;
                    } else {
                        y--;
                        d += incr2;
                    }
                    line.points.add(cnt, new Point<Integer>(x, y));
                    cnt++;
                }
            }
        } else {
            d = 2 * dx - dy;
            incr1 = 2 * dx;
            incr2 = 2 * (dx - dy);
            if (start.y > end.y) {
                y = end.y;
                x = end.x;
                yend = start.y;
                xdirflag = (-1);
            } else {
                y = start.y;
                x = start.x;
                yend = end.y;
                xdirflag = 1;
            }
            line.points.add(cnt, new Point<Integer>(x, y));
            cnt++;
            if (((end.x - start.x) * xdirflag) > 0) {
                while (y < yend) {
                    y++;
                    if (d < 0) {
                        d += incr1;
                    } else {
                        x++;
                        d += incr2;
                    }
                    line.points.add(cnt, new Point<Integer>(x, y));
                    cnt++;
                }
            } else {
                while (y < yend) {
                    y++;
                    if (d < 0) {
                        d += incr1;
                    } else {
                        x--;
                        d += incr2;
                    }
                    line.points.add(cnt, new Point<Integer>(x, y));
                    cnt++;
                }
            }
        }
    }


    public static void gridLine(Point<Integer> start, Point<Integer> end, GridLineTraversalLine line) {
        int i, j;
        int half;
        Point<Integer> v;
        gridLineCore(start, end, line);
        if (!start.x.equals(line.points.get(0).x) ||
                !start.y.equals(line.points.get(0).y)) {
            half = line.points.size() / 2;
            for (i = 0, j = line.points.size() - 1; i < half; i++, j--) {
                v = line.points.get(i);
                line.points.set(i, line.points.get(j));
                line.points.set(j, v);
            }
        }
    }
}
