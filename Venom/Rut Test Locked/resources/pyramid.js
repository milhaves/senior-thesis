function PyramidClass(height, width ){
  this.height = height
  this.width = width
  this.hwidth
  
  function createVertices(){
    let hwidth = this.hwidth
    this.vertices = [[-hwidth,hwidth,0],[hwidth,hwidth,0],[hwidth,-hwidth,0],[-hwidth,-hwidth,0],[0,0,this.height]]
  }
  
  function createIndices(){
    this.createVertices()
    this.coordIndices = [[0,1,3],[1,2,3],[0,3,4],[1,2,4],[2,3,4]]
  }


}