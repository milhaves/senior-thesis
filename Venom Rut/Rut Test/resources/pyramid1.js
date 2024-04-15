function PyramidClass(height, width ){
    this.height = height
    this.width = width
    this.hwidth
    
   this.createVertices = function(){
      let hwidth = this.hwidth
      this.vertices = [[-hwidth,hwidth,0],[hwidth,hwidth,0],[hwidth,-hwidth,0],[-hwidth,-hwidth,0],[0,0,this.height]]
    }
    
    this.createIndices = function(){
      this.createVertices()
      this.coordIndices = [[0,1,3],[1,2,3],[0,3,4],[1,2,4],[2,3,4]]
    }
  }
//creating a pyramid object
var myPyr = PyramidClass();
//create our coords/indices
myPyr.createIndices(3,4);
console.log(myPyr.coordIndices)