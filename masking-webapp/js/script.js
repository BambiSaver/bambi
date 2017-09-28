/**
 * Setup the canvas element
 */
function setup()
{
  
  w = window.innerWidth;
  h = window.innerHeight - $("#mainMenu").first().outerHeight();

  var canvas_wrapper = document.getElementById('canvas-wrapper');
  canvas_wrapper.innerHTML = '';
  canvas_wrapper.innerHTML += '<canvas id="board" width="' + w + '" height="'+h+'"></canvas>';
  //canvas_wrapper.innerHTML += '<div class="dimensions">' + w + 'px x ' + h + 'px</div>';

  canvas = document.getElementById('board');
	c = canvas.getContext('2d');
 
  /*
  renderer = setInterval(function()
  {
    render();
  }, 40);*/
}

/*
var render = function()
{
  c.fillStyle = 'rgba(180, 180, 180, 0.6)';
	c.fillRect(0,0,w,h);
}*/

function resized()
{
  //clearInterval(renderer);
  setup();
}
/**
 * Run the setup on launch and add event listener
 * to rerun the setup if the window resizes.
 */
setup();
window.addEventListener( 'resize', resized );




// CANVAS DRAWING APP


var clickX = new Array();
var clickY = new Array();
var clickDrag = new Array();
var paint;

function addClick(x, y, dragging)
{
  clickX.push(x);
  clickY.push(y);
  clickDrag.push(dragging);
}

function redraw(){
  c.clearRect(0, 0, c.canvas.width, c.canvas.height); // Clears the canvas
  
  c.strokeStyle = "#df4b26";
  c.lineJoin = "round";
  c.lineWidth = 10;
      
  for(var i=0; i < clickX.length; i++) {    
    c.beginPath();
    if(clickDrag[i] && i){
      c.moveTo(clickX[i-1], clickY[i-1]);
     }else{
       c.moveTo(clickX[i]-1, clickY[i]);
     }
     c.lineTo(clickX[i], clickY[i]);
     c.closePath();
     c.stroke();
  }
}



$(canvas).mousedown(function(e){
  var mouseX = e.pageX - this.offsetLeft;
  var mouseY = e.pageY - this.offsetTop;
    
  paint = true;
  addClick(mouseX, mouseY);
  redraw();
});

$(canvas).mousemove(function(e){
  if(paint){
    var mouseX = e.pageX - this.offsetLeft;
    var mouseY = e.pageY - this.offsetTop;
    addClick(mouseX, mouseY, true);
    redraw();
  }
});



$(canvas).mouseup(function(e){
  paint = false;
});


$(canvas).mouseleave(function(e){
  paint = false;
});

function callback(response) {
  alert("The server says: " + response);
}

$('#maskSubmitForm').on('submit', function (e) {
    dataURL = canvas.toDataURL();

    $.ajax({
      type: "POST",
      url: "upload-canvas.php",
      data: { 
         imgBase64: dataURL
      },
      success: function (response) {
          alert("The server says: " + response);
      }
    }).done(function(o) {
      console.log(dataURL); 
    });

    e.preventDefault(); 
});