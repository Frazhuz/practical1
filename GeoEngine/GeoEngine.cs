namespace GeoEngine;

public static class GeoEngine
{
    private const double DefaultTolerance = 1e-10;

    /// <summary>
    /// Вычисляет скалярное произведение двух векторов.
    /// </summary>
    public static double DotProduct(Vector v1, Vector v2)
    {
        ArgumentNullException.ThrowIfNull(v1);
        ArgumentNullException.ThrowIfNull(v2);

        return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
    }

    /// <summary>
    /// Вычисляет векторное произведение двух векторов.
    /// </summary>
    public static Vector CrossProduct(Vector v1, Vector v2)
    {
        ArgumentNullException.ThrowIfNull(v1);
        ArgumentNullException.ThrowIfNull(v2);

        double x = v1.Y * v2.Z - v1.Z * v2.Y;
        double y = v1.Z * v2.X - v1.X * v2.Z;
        double z = v1.X * v2.Y - v1.Y * v2.X;

        return new Vector(x, y, z);
    }

    /// <summary>
    /// Вычисляет угол между двумя векторами в радианах.
    /// </summary>
    public static double AngleBetween(Vector v1, Vector v2)
    {
        ArgumentNullException.ThrowIfNull(v1);
        ArgumentNullException.ThrowIfNull(v2);

        double dot = DotProduct(v1, v2);
        double lengths = v1.Length * v2.Length;

        if (lengths < DefaultTolerance)
        {
            throw new ArgumentException("Один из векторов имеет нулевую длину");
        }

        double cos = Math.Clamp(dot / lengths, -1, 1);

        return Math.Acos(cos);
    }

    /// <summary>
    /// Проверяет, являются ли векторы коллинеарными.
    /// </summary>
    public static bool AreCollinear(Vector v1, Vector v2, double tolerance = DefaultTolerance)
    {
        ArgumentNullException.ThrowIfNull(v1);
        ArgumentNullException.ThrowIfNull(v2);

        var cross = CrossProduct(v1, v2);
        return Math.Abs(cross.X) < tolerance &&
               Math.Abs(cross.Y) < tolerance &&
               Math.Abs(cross.Z) < tolerance;
    }

    /// <summary>
    /// Проверяет принадлежность точки выпуклому многограннику.
    /// Грани задаются наборами вершин; точка на границе считается принадлежащей.
    /// </summary>
    public static bool IsPointInsideConvexPolyhedron(
        Point point,
        IEnumerable<IEnumerable<Point>> faces,
        double tolerance = DefaultTolerance)
    {
        ArgumentNullException.ThrowIfNull(point);
        ArgumentNullException.ThrowIfNull(faces);

        var faceList = faces
            .Select(face => face?.ToArray() ?? throw new ArgumentException("Грань не может быть null.", nameof(faces)))
            .ToArray();
        if (faceList.Length == 0)
        {
            throw new ArgumentException("Многогранник должен содержать хотя бы одну грань.", nameof(faces));
        }

        var vertices = faceList.SelectMany(face => face).Distinct().ToArray();
        if (vertices.Length < 4)
        {
            throw new ArgumentException("Многогранник должен содержать хотя бы четыре вершины.", nameof(faces));
        }

        foreach (var face in faceList)
        {
            if (face.Length < 3)
            {
                throw new ArgumentException("Каждая грань должна содержать хотя бы три вершины.", nameof(faces));
            }

            var normal = GetFaceNormal(face, tolerance);
            double pointDistance = DotProduct(normal, new Vector(face[0], point));
            double? referenceDistance = vertices
                .Where(vertex => !face.Contains(vertex))
                .Select(vertex => DotProduct(normal, new Vector(face[0], vertex)))
                .Where(distance => Math.Abs(distance) > tolerance)
                .Cast<double?>()
                .FirstOrDefault();

            if (referenceDistance is null)
            {
                throw new ArgumentException("Грани не описывают объемный выпуклый многогранник.", nameof(faces));
            }

            if (pointDistance * referenceDistance.Value < -tolerance)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Вычисляет кратчайшее расстояние между двумя отрезками в пространстве.
    /// </summary>
    public static double DistanceBetweenSegments(Point firstStart, Point firstEnd, Point secondStart, Point secondEnd)
    {
        ArgumentNullException.ThrowIfNull(firstStart);
        ArgumentNullException.ThrowIfNull(firstEnd);
        ArgumentNullException.ThrowIfNull(secondStart);
        ArgumentNullException.ThrowIfNull(secondEnd);

        var firstDirection = new Vector(firstStart, firstEnd);
        var secondDirection = new Vector(secondStart, secondEnd);
        var betweenStarts = new Vector(secondStart, firstStart);

        double a = DotProduct(firstDirection, firstDirection);
        double e = DotProduct(secondDirection, secondDirection);
        double f = DotProduct(secondDirection, betweenStarts);

        if (a <= DefaultTolerance && e <= DefaultTolerance)
        {
            return Distance(firstStart, secondStart);
        }

        double s;
        double t;
        if (a <= DefaultTolerance)
        {
            s = 0;
            t = Math.Clamp(f / e, 0, 1);
        }
        else
        {
            double c = DotProduct(firstDirection, betweenStarts);
            if (e <= DefaultTolerance)
            {
                t = 0;
                s = Math.Clamp(-c / a, 0, 1);
            }
            else
            {
                double b = DotProduct(firstDirection, secondDirection);
                double denominator = a * e - b * b;

                s = denominator > DefaultTolerance
                    ? Math.Clamp((b * f - c * e) / denominator, 0, 1)
                    : 0;

                t = (b * s + f) / e;
                if (t < 0)
                {
                    t = 0;
                    s = Math.Clamp(-c / a, 0, 1);
                }
                else if (t > 1)
                {
                    t = 1;
                    s = Math.Clamp((b - c) / a, 0, 1);
                }
            }
        }

        var closestFirst = Add(firstStart, Scale(firstDirection, s));
        var closestSecond = Add(secondStart, Scale(secondDirection, t));

        return Distance(closestFirst, closestSecond);
    }

    /// <summary>
    /// Вычисляет осевой момент инерции множества точек относительно заданной оси.
    /// </summary>
    public static double CalculateAxialMomentOfInertia(
        IEnumerable<Point> points,
        Vector axisDirection,
        Point? axisPoint = null,
        double massPerPoint = 1)
    {
        ArgumentNullException.ThrowIfNull(axisDirection);
        ValidateMass(massPerPoint);

        var pointList = MaterializePoints(points);
        if (axisDirection.Length <= DefaultTolerance)
        {
            throw new ArgumentException("Направление оси не может быть нулевым.", nameof(axisDirection));
        }

        Point origin = axisPoint ?? new Point(0, 0, 0);
        double axisLengthSquared = DotProduct(axisDirection, axisDirection);

        return pointList.Sum(point =>
        {
            var originToPoint = new Vector(origin, point);
            var cross = CrossProduct(originToPoint, axisDirection);
            double distanceSquared = DotProduct(cross, cross) / axisLengthSquared;
            return massPerPoint * distanceSquared;
        });
    }

    /// <summary>
    /// Вычисляет геометрический момент инерции множества точек относительно заданной точки.
    /// </summary>
    public static double CalculateGeometricMomentOfInertia(
        IEnumerable<Point> points,
        Point? referencePoint = null,
        double massPerPoint = 1)
    {
        ValidateMass(massPerPoint);

        var pointList = MaterializePoints(points);
        Point origin = referencePoint ?? new Point(0, 0, 0);

        return pointList.Sum(point =>
        {
            var vector = new Vector(origin, point);
            return massPerPoint * DotProduct(vector, vector);
        });
    }

    private static Vector GetFaceNormal(IReadOnlyList<Point> face, double tolerance)
    {
        for (int i = 1; i < face.Count - 1; i++)
        {
            var first = new Vector(face[0], face[i]);
            var second = new Vector(face[0], face[i + 1]);
            var normal = CrossProduct(first, second);
            if (normal.Length > tolerance)
            {
                return normal;
            }
        }

        throw new ArgumentException("Грань должна содержать неколлинеарные вершины.", nameof(face));
    }

    private static Point[] MaterializePoints(IEnumerable<Point> points)
    {
        ArgumentNullException.ThrowIfNull(points);

        var pointList = points.ToArray();
        if (pointList.Length == 0)
        {
            throw new ArgumentException("Множество точек не может быть пустым.", nameof(points));
        }

        if (pointList.Any(point => point is null))
        {
            throw new ArgumentException("Множество точек не может содержать null.", nameof(points));
        }

        return pointList!;
    }

    private static void ValidateMass(double massPerPoint)
    {
        if (massPerPoint < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(massPerPoint), "Масса точки не может быть отрицательной.");
        }
    }

    private static Point Add(Point point, Vector vector)
    {
        return new Point(point.X + vector.X, point.Y + vector.Y, point.Z + vector.Z);
    }

    private static Vector Scale(Vector vector, double multiplier)
    {
        return new Vector(vector.X * multiplier, vector.Y * multiplier, vector.Z * multiplier);
    }

    private static double Distance(Point first, Point second)
    {
        return new Vector(first, second).Length;
    }
}
