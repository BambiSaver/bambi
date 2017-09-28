/**
 * Setup the canvas element
 */
function setup()
{
  w = window.innerWidth;
  h = window.innerHeight;
  size = 40;
  var canvas_wrapper = document.getElementById('canvas-wrapper');
  canvas_wrapper.innerHTML = '';
  canvas_wrapper.innerHTML += '<canvas id="board" width="' + w + '" height="'+h+'"></canvas>';
  canvas_wrapper.innerHTML += '<div class="dimensions">' + w + 'px x ' + h + 'px</div>';
  var canvas = document.getElementById('board');
	c = canvas.getContext('2d');
 
  renderer = setInterval(function()
  {
    render();
  }, 40);
}

var render = function()
{
  c.fillStyle = 'rgba(17, 17, 17, 0.6)';
	c.fillRect(0,0,w,h);
}

function resized()
{
  clearInterval(renderer);
  setup();
}
/**
 * Run the setup on launch and add event listener
 * to rerun the setup if the window resizes.
 */
setup();
window.addEventListener( 'resize', resized );
