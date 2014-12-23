package cgl.iotrobots.collavoid.rosplanners.utils;


public class utils {


    public static Vector2 projectPointOnLine(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        double r = (Vector2.dotProduct(Vector2.minus(point, pointLine), dirLine)) / Vector2.absSqr(dirLine);
        return Vector2.plus(pointLine, Vector2.mul(dirLine, r));
    }

    /*!
     *  @brief      Computes the squared distance from a line segment with the
     *              specified endpoints to a specified point.
     *  @param      a               The first endpoint of the line segment.
     *  @param      b               The second endpoint of the line segment.
     *  @param      c               The point to which the squared distance is to
     *                              be calculated.
     *  @returns    The squared distance from the line segment to the point.
     */
    public static double distSqPointLineSegment(final Vector2 a, final Vector2 b, final Vector2 c) {
        final double r = Vector2.dotProduct(Vector2.minus(c, a), Vector2.minus(b, a)) / Vector2.absSqr(Vector2.minus(b, a));

        if (r < 0.0f) {
            return Vector2.absSqr(Vector2.minus(c, a));
        } else if (r > 1.0f) {
            return Vector2.absSqr(Vector2.minus(c, b));
        } else {
            return Vector2.absSqr(Vector2.minus(c, Vector2.plus(a, Vector2.mul(Vector2.minus(b, a), r))));
        }
    }

    /*!
     *  @brief      Computes the sign from a line connecting the
     *              specified points to a specified point.
     *  @param      a               The first point on the line.
     *  @param      b               The second point on the line.
     *  @param      c               The point to which the signed distance is to
     *                              be calculated.
     *  @returns    Positive when the point c lies to the left of the line ab.
     */
    public static double signedDistPointToLineSegment(final Vector2 a, final Vector2 b, final Vector2 c) {
        return Vector2.det(Vector2.minus(a, c), Vector2.minus(b, a));
    }

    public static double left(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point);
    }

    public static boolean leftOf(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point) > EPSILON.EPSILON;
    }

    public static boolean rightOf(final Vector2 pointLine, final Vector2 dirLine, final Vector2 point) {
        return signedDistPointToLineSegment(pointLine, Vector2.plus(pointLine, dirLine), point) < -EPSILON.EPSILON;
    }

    public static double sign(double x) {
        return x < 0.0 ? -1.0 : 1.0;
    }

}
