function straightRutRoad(rLen, dx, rW, rut_radius, rut_N ){
  this.rLen = rLen;
  this.dx = dx;
  this.rW = rW;
  this.rut_radius = rut_radius;
  this.rut_N = rut_N;
  this.vertices = []//this will hold all xyz points of the road
  this.coordIndices = []//this is a list of triangles defined by the indices in the 'vertices' list.
  
  this.createSlice = function(xnow){
    //create y-z slice of road points. start empty
    let roadslice = [];
    //push adds something to a list. Add the left edge.
    //each point will be a list in and of itself... [x,y,z]
    roadslice.push([xnow,this.rW/2,0]);
    //now add the points in the rut itself.
    var dtheta = Math.PI/this.rut_N
    //now actually make points for the rut itself. It's a semicircle!
    for(let i=0;i<this.rut_N+1;i++){
      var ynow = this.rut_radius*Math.cos(-i*dtheta);
      var znow = this.rut_radius*Math.sin(-i*dtheta);
      //add this vector of points to the list of vertices in this 'slice' of road.
      roadslice.push([xnow,ynow,znow]);
    }  
    //now add the right edge
    roadslice.push([xnow,-this.rW/2,0])
    //this function returns the 'slice' of the road, a list of 3-vectors describing each vertex.
    return roadslice;
  }
  //this function creates ALL the vertices for ALL the slices in our road segment.
  this.createVertices = function(){
    //start at 0
    //xnow = 0;
    //create a vector to keep track of how many slices we have. could pre-calculate this, but eh.
    this.num_slices = 0
    for(let xnow = 0;xnow<=(this.rLen/this.dx);xnow=xnow+this.dx){
      //increment number of slices (we'll need this later for triangles.)
        this.num_slices++;
      //use our function above to create the slice
        var myslice = this.createSlice(xnow)
        //add the points in this slice to our list.
        this.vertices=this.vertices.concat(myslice)
    }
  }
  
  this.createTriangles = function(){
    //first create vertices of our road using the function above.
    this.createVertices();
    //now, we have to define a list of triangles, defined by sets of three indices in our vertex array that are connected. like [0,1,2] would mean that vertices 0, 1, and 2 form a triangle. 
    
    //outer for-loop that moves through each slice.
    for(let slice_ind = 0;slice_ind<(this.num_slices-1);slice_ind++){
      //inner for-loop that moves through each triangle in the slice.
      //we will always make triangles from the current slice TO the next.
      //for any point, triangle 1 is defined by 2 points on next slice, and this slice.
      //for any point, triangle 2 is defined by this point, the point to the right, and then one point on the next slice.
      
      //first, the for-loop defining 'triangle 1' for each point (except the last one, which is only a member of triangle 2).
      for(let point_ind = 0;point_ind<(this.rut_N+2);point_ind++){
        //offset the index based on which slice we're working on. on second slice (index 1), the total number of points in one slice must be added to get to the 0'th index for this slice.
        var offsetThis = slice_ind*(this.rut_N+3)
        //same thing; find an offset that gets us to the same point in the NEXT slice
        var offsetNext = (slice_ind+1)*(this.rut_N+3)
        //this triangle is me, me on the next slice, and neighbor on next slice.
        //have to be defined in CCW order!
        var tri = [offsetThis+point_ind,offsetNext+point_ind+1,offsetNext+point_ind]
        this.coordIndices.push(tri)
      
      //now repeat this, but use the pattern: me, my neighbor, and my neighbor on the NEXT slice. This gets us full coverage.
        //offset the index based on which slice we're working on. on second slice (index 1), the total number of points in one slice must be added to get to the 0'th index for this slice.
        var offsetThis = slice_ind*(this.rut_N+3)
        //same thing; find an offset that gets us to the same point in the NEXT slice
        var offsetNext = (slice_ind+1)*(this.rut_N+3)
        //this triangle is me, me on the next slice, and neighbor on next slice.
        //have to be defined in CCW order!
        var tri = [offsetThis+point_ind,offsetThis+point_ind+1,offsetNext+point_ind+1]
        this.coordIndices.push(tri)
      }
      
    }
    
  }
  
}