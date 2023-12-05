function generateRoadData() {
    var myRoad = new straightRutRoad(5, 1, 5, 0.1, 10);
    myRoad.createTriangles();

    var coordArray = [];
    for (var i = 0; i < myRoad.vertices.length; i++) {
        coordArray.push(myRoad.vertices[i][0], myRoad.vertices[i][1], myRoad.vertices[i][2]);
    }

    RoadDefinition.vertices = coordArray;
    RoadDefinition.coordIndices = myRoad.coordIndices;
}

generateRoadData();
