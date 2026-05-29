namespace GeoEngine.Tests;

[TestFixture]
public class GeoEngineTests
{
    private const double Tolerance = 1e-10;

    [Test]
    public void DotProduct_OrthogonalVectors_ReturnsZero()
    {
        var result = GeoEngine.DotProduct(new Vector(1, 0, 0), new Vector(0, 1, 0));

        Assert.That(result, Is.EqualTo(0));
    }

    [Test]
    public void DotProduct_GeneralVectors_ReturnsSumOfCoordinateProducts()
    {
        var result = GeoEngine.DotProduct(new Vector(1, -2, 3), new Vector(4, 5, -6));

        Assert.That(result, Is.EqualTo(-24));
    }

    [Test]
    public void DotProduct_ZeroVector_ReturnsZero()
    {
        var result = GeoEngine.DotProduct(new Vector(0, 0, 0), new Vector(4, 5, 6));

        Assert.That(result, Is.EqualTo(0));
    }

    [Test]
    public void CrossProduct_StandardBasisVectors_ReturnsThirdBasisVector()
    {
        var result = GeoEngine.CrossProduct(new Vector(1, 0, 0), new Vector(0, 1, 0));

        Assert.That(result, Is.EqualTo(new Vector(0, 0, 1)));
    }

    [Test]
    public void CrossProduct_ReversedBasisVectors_ReturnsNegativeThirdBasisVector()
    {
        var result = GeoEngine.CrossProduct(new Vector(0, 1, 0), new Vector(1, 0, 0));

        Assert.That(result, Is.EqualTo(new Vector(0, 0, -1)));
    }

    [Test]
    public void CrossProduct_ParallelVectors_ReturnsZeroVector()
    {
        var result = GeoEngine.CrossProduct(new Vector(1, 2, 3), new Vector(2, 4, 6));

        Assert.That(result, Is.EqualTo(new Vector(0, 0, 0)));
    }

    [Test]
    public void AngleBetween_ParallelVectors_ReturnsZero()
    {
        var angle = GeoEngine.AngleBetween(new Vector(1, 0, 0), new Vector(2, 0, 0));

        Assert.That(angle, Is.EqualTo(0).Within(Tolerance));
    }

    [Test]
    public void AngleBetween_OrthogonalVectors_ReturnsHalfPi()
    {
        var angle = GeoEngine.AngleBetween(new Vector(1, 0, 0), new Vector(0, 3, 0));

        Assert.That(angle, Is.EqualTo(Math.PI / 2).Within(Tolerance));
    }

    [Test]
    public void AngleBetween_ZeroLengthVector_ThrowsException()
    {
        Assert.Throws<ArgumentException>(() => GeoEngine.AngleBetween(new Vector(1, 0, 0), new Vector(0, 0, 0)));
    }

    [Test]
    public void AreCollinear_ParallelVectors_ReturnsTrue()
    {
        Assert.That(GeoEngine.AreCollinear(new Vector(1, 2, 3), new Vector(2, 4, 6)), Is.True);
    }

    [Test]
    public void AreCollinear_OppositeVectors_ReturnsTrue()
    {
        Assert.That(GeoEngine.AreCollinear(new Vector(1, -2, 3), new Vector(-2, 4, -6)), Is.True);
    }

    [Test]
    public void AreCollinear_NonCollinearVectors_ReturnsFalse()
    {
        Assert.That(GeoEngine.AreCollinear(new Vector(1, 2, 3), new Vector(1, 2, 4)), Is.False);
    }

    [Test]
    public void IsPointInsideConvexPolyhedron_PointInsideCube_ReturnsTrue()
    {
        Assert.That(GeoEngine.IsPointInsideConvexPolyhedron(new Point(0.5, 0.5, 0.5), CreateUnitCubeFaces()), Is.True);
    }

    [Test]
    public void IsPointInsideConvexPolyhedron_PointOnCubeFace_ReturnsTrue()
    {
        Assert.That(GeoEngine.IsPointInsideConvexPolyhedron(new Point(0, 0.5, 0.5), CreateUnitCubeFaces()), Is.True);
    }

    [Test]
    public void IsPointInsideConvexPolyhedron_PointOutsideCube_ReturnsFalse()
    {
        Assert.That(GeoEngine.IsPointInsideConvexPolyhedron(new Point(1.5, 0.5, 0.5), CreateUnitCubeFaces()), Is.False);
    }

    [Test]
    public void DistanceBetweenSegments_IntersectingSegments_ReturnsZero()
    {
        double distance = GeoEngine.DistanceBetweenSegments(
            new Point(0, 0, 0),
            new Point(1, 0, 0),
            new Point(0.5, -1, 0),
            new Point(0.5, 1, 0));

        Assert.That(distance, Is.EqualTo(0).Within(Tolerance));
    }

    [Test]
    public void DistanceBetweenSegments_ParallelSegments_ReturnsPerpendicularDistance()
    {
        double distance = GeoEngine.DistanceBetweenSegments(
            new Point(0, 0, 0),
            new Point(1, 0, 0),
            new Point(0, 2, 0),
            new Point(1, 2, 0));

        Assert.That(distance, Is.EqualTo(2).Within(Tolerance));
    }

    [Test]
    public void DistanceBetweenSegments_DegenerateFirstSegment_ReturnsPointToSegmentDistance()
    {
        double distance = GeoEngine.DistanceBetweenSegments(
            new Point(0, 0, 0),
            new Point(0, 0, 0),
            new Point(3, 4, 0),
            new Point(3, 4, 5));

        Assert.That(distance, Is.EqualTo(5).Within(Tolerance));
    }

    [Test]
    public void CalculateAxialMomentOfInertia_AroundZAxis_ReturnsSumOfSquaredDistances()
    {
        var points = new[] { new Point(1, 0, 0), new Point(0, 2, 0) };

        double moment = GeoEngine.CalculateAxialMomentOfInertia(points, new Vector(0, 0, 1));

        Assert.That(moment, Is.EqualTo(5).Within(Tolerance));
    }

    [Test]
    public void CalculateAxialMomentOfInertia_WithShiftedAxis_UsesAxisPoint()
    {
        var points = new[] { new Point(1, 5, 0) };

        double moment = GeoEngine.CalculateAxialMomentOfInertia(points, new Vector(0, 0, 2), new Point(1, 0, 0));

        Assert.That(moment, Is.EqualTo(25).Within(Tolerance));
    }

    [Test]
    public void CalculateAxialMomentOfInertia_ZeroAxis_ThrowsException()
    {
        var points = new[] { new Point(1, 0, 0) };

        Assert.Throws<ArgumentException>(() => GeoEngine.CalculateAxialMomentOfInertia(points, new Vector(0, 0, 0)));
    }

    [Test]
    public void CalculateGeometricMomentOfInertia_FromOrigin_ReturnsSumOfSquaredDistances()
    {
        var points = new[] { new Point(1, 0, 0), new Point(0, 2, 0) };

        double moment = GeoEngine.CalculateGeometricMomentOfInertia(points);

        Assert.That(moment, Is.EqualTo(5).Within(Tolerance));
    }

    [Test]
    public void CalculateGeometricMomentOfInertia_WithReferencePoint_UsesReferencePoint()
    {
        var points = new[] { new Point(1, 1, 0), new Point(2, 1, 0) };

        double moment = GeoEngine.CalculateGeometricMomentOfInertia(points, new Point(1, 1, 0));

        Assert.That(moment, Is.EqualTo(1).Within(Tolerance));
    }

    [Test]
    public void CalculateGeometricMomentOfInertia_WithMassPerPoint_AppliesMass()
    {
        var points = new[] { new Point(1, 0, 0), new Point(0, 2, 0) };

        double moment = GeoEngine.CalculateGeometricMomentOfInertia(points, massPerPoint: 2);

        Assert.That(moment, Is.EqualTo(10).Within(Tolerance));
    }

    private static Point[][] CreateUnitCubeFaces()
    {
        var p000 = new Point(0, 0, 0);
        var p100 = new Point(1, 0, 0);
        var p010 = new Point(0, 1, 0);
        var p110 = new Point(1, 1, 0);
        var p001 = new Point(0, 0, 1);
        var p101 = new Point(1, 0, 1);
        var p011 = new Point(0, 1, 1);
        var p111 = new Point(1, 1, 1);

        return
        [
            [p000, p100, p110, p010],
            [p001, p101, p111, p011],
            [p000, p100, p101, p001],
            [p010, p110, p111, p011],
            [p000, p010, p011, p001],
            [p100, p110, p111, p101],
        ];
    }
}
