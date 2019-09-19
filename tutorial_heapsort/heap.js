/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

  var elntIdx=heap.length;
  var prntIdx=Math.floor((elntIdx-1)/2);
  heap.push(new_element);
  var heaped=(elntIdx<=0)||(heap[prntIdx]<=heap[elntIdx]);
  while(!heaped){
    var tmp=heap[prntIdx];
    heap[prntIdx]=heap[elntIdx];
    heap[elntIdx]=tmp;
    elntIdx=prntIdx;
    prntIdx=Math.floor((elntIdx-1)/2);
    heaped=(elntIdx<=0)||(heap[prntIdx]<=heap[elntIdx]);
  }


    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    var takeout=heap[0]; 
    var stat=0; //stat means cureent position; after comparison stat will change
    heap[stat]=heap[heap.length-1]; 
    heap.pop(heap.length-1);//delete the last element
    while(stat<heap.length)
    { 
        var tmp=heap[stat]; 
        child_No1=2*stat+1; 
        child_No2=2*stat+2;

        //first compare wthich is smallest among three 
        if(child_No1<=heap.length-1){ 
            if(tmp>heap[child_No1])
                {min=heap[child_No1];}
            else{min=tmp;} 
        } 

        else{break;} 
        if(child_No2<=heap.length-1){ 
            if(min>heap[child_No2])
            {min=heap[child_No2];} 
        } 
        if(min==tmp){break;} 

        // now change the position
        if(min==heap[child_No1]){ 
            heap[stat]=heap[child_No1];
            heap[child_No1]=tmp; 
            stat=child_No1; 
        }
        else{ 
            heap[stat]=heap[child_No2]; 
            heap[child_No2]=tmp; 
            stat=child_No2; 
        } 
    } 
    return takeout; 
}
    // STENCIL: implement your min binary heap extract operation
    

        
       

// assign extract function within minheaper object
minheaper.extract = minheap_extract;