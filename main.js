var canvas = document.getElementById("myCanvas");
var ctx = canvas.getContext("2d");
var world = createWorld();


var loader = new b2Loader();

fetch('scene.json').then(response => response.json()).then(function(scene){
	loader.loadScene(scene, world);

	var draw = new Draw(canvas, ctx, loader.get());
		draw.onkeydown();
		balance(loader.get())
	function update(){
	   draw.update(world);
	}	

	window.setInterval(update, 1000/60);
});
