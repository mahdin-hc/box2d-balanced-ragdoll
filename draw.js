function Navigator(){
	
	this.panning = [0, 100];				
	this.origin = [0, 0];						
	this.scale = 1;								
	this.scaleLimits = [0.05, 16];				
						
}

Navigator.prototype.screenPointToWorld = function(x, y){
	return [(x / this.scale - this.panning[0] + this.origin[0]), (y / this.scale - this.panning[1] + this.origin[1])];
};

Navigator.prototype.worldPointToScreen = function(x, y){
	return [(x + this.panning[0] - this.origin[0]) * this.scale, (y + this.panning[1] - this.origin[1]) * this.scale];	
};


var mousePVec;
var isMouseDown;
var mouseJoint;
var selectedBody;

function Draw(canvas, ctx, loadedScene){
	this.keyPressed;
	this.loadedScene = loadedScene;
	this.scale = 30;
	this.clearColor = "#333333"
	this.canvas = canvas;
	this.ctx = ctx;
	this.navigator = new Navigator();
	
	this.width = canvas.width;
	this.height = canvas.height;
	
	this.mouseX;
	this.mouseY;
	var ref = this;
	
	//mobile
	ref.canvas.ontouchmove = function(e){
		isMouseDown = true;
		ref.handleMouseMove(e.touches[0].clientX, e.touches[0].clientY);
	}
	ref.canvas.ontouchstart = function(e){
		ref.handleMouseMove(e.touches[0].clientX, e.touches[0].clientY);
	};
	ref.canvas.ontouchend = function(e){
		isMouseDown = false;
	};
	//desktop
	ref.canvas.addEventListener('mousedown', (e) => {
		isMouseDown = true;
		ref.handleMouseMove(e.offsetX, e.offsetY);
	}, true);

	ref.canvas.addEventListener('mousemove', (e) => {
		ref.handleMouseMove(e.offsetX, e.offsetY) 
	}, true);

	ref.canvas.addEventListener('mouseup', () => {
		isMouseDown = false;
	}, true);
	
	
	ref.zoom(this.canvas.width / 2, this.canvas.height / 2, 1);
	
}


Draw.prototype.updateMouse = function(world){

	if(isMouseDown && (!mouseJoint)){
		var body = this.getBodyAtMouse(world);
		if(body) {
			var md = new b2MouseJointDef();
			md.bodyA = world.CreateBody(new box2d.b2BodyDef());
			md.bodyB = body;
			md.target = new b2Vec2(this.mouseX, this.mouseY);
			md.collideConnected = true;
			md.maxForce = 1000.0 * body.GetMass();
			mouseJoint = world.CreateJoint(md);
			body.SetAwake(true);
		}
	}
	if(mouseJoint){
		if(isMouseDown) {
			mouseJoint.SetTarget(new b2Vec2(this.mouseX, this.mouseY));
		} 
		else {
			world.DestroyJoint(mouseJoint);
			mouseJoint = null;
		}
	}

};

Draw.prototype.getBodyAtMouse = function(world) {
	mousePVec = new b2Vec2(this.mouseX, this.mouseY);
	var aabb = new b2AABB();

	aabb.lowerBound = new b2Vec2(this.mouseX - 0.001, this.mouseY - 0.001);
	aabb.upperBound = new b2Vec2(this.mouseX + 0.001, this.mouseY + 0.001);

	// Query the world for overlapping shapes.
	selectedBody = null;
	world.QueryAABB(this.getBodyCB, aabb);
	return selectedBody;
}

Draw.prototype.getBodyCB = function(fixture) {

	if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
		if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), this.mousePVec)) {
			selectedBody = fixture.GetBody();
			return false;
		}
	}
	return true;
}


Draw.prototype.handleMouseMove = function(x, y){ 
	this.mouseX = this.navigator.screenPointToWorld(x, y)[0] / 30;
	this.mouseY = this.navigator.screenPointToWorld(x, y)[1] / 30;
}



Draw.prototype.onkeydown = function(){
	
	window.addEventListener('keydown', (event) => {  

		this.keyPressed = event.key;

		if(event.key == 'ArrowRight'){
			this.navigator.panning[0] -= 10;
		}
		else if(event.key == 'ArrowLeft'){
			this.navigator.panning[0] += 10;
		}
		else if(event.key == 'ArrowUp'){
			this.navigator.panning[1] += 10;
		}	
		else if(event.key == 'ArrowDown'){
			this.navigator.panning[1] -= 10;	
		}
		else if(event.key == 'z'){
			this.zoom(this.canvas.width / 2, this.canvas.height / 2, 1.2);
		} 
		else if(event.key == 'x'){
			this.zoom(this.canvas.width / 2, this.canvas.height / 2, 0.8);
		} 
	});
}
Draw.prototype.clear = function(x, y, w, h){
		this.ctx.fillStyle = this.clearColor;
		this.ctx.clearRect(x, y, w, h);
		this.ctx.fillRect(x, y, w, h);
	};

Draw.prototype.lookAt = function(body, x = 0, y = 0){
	
	
	var pos = body.GetPosition();
	var ps = this.navigator.worldPointToScreen(pos.x,pos.y)
	
	this.navigator.panning[0] = (-pos.x * this.scale)+x;
	this.navigator.panning[1] = (-pos.y * this.scale)+y;
	
	
}
	
Draw.prototype.zoom = function(mouseX, mouseY, zoom){
	var navigator = this.navigator;
	
	if (zoom > 1){
		if (navigator.scale > navigator.scaleLimits[1])
			return;
	}
	else{
		if (navigator.scale < navigator.scaleLimits[0]) 
			return;
	}
	this.ctx.translate(
		navigator.origin[0],
		navigator.origin[1]
	);
	this.ctx.scale(zoom,zoom);
	this.ctx.translate(
		-( mouseX / navigator.scale + navigator.origin[0] - mouseX / ( navigator.scale * zoom ) ),
		-( mouseY / navigator.scale + navigator.origin[1] - mouseY / ( navigator.scale * zoom ) )
	);
	navigator.origin[0] = ( mouseX / navigator.scale + navigator.origin[0] - mouseX / ( navigator.scale * zoom ) );
	navigator.origin[1] = ( mouseY / navigator.scale + navigator.origin[1] - mouseY / ( navigator.scale * zoom ) );
	navigator.scale *= zoom;
};

var lastLoop = new Date();

Draw.prototype.update = function(world){
    var thisLoop = new Date();
    var fps = 1000 / (thisLoop - lastLoop);
    lastLoop = thisLoop;
	
	this.updateMouse(world);
	world.Step(1.0 / 60, 10, 10);
	var navigator = this.navigator
	this.lookAt(this.loadedScene.bodies[0], this.width/2,this.height/2);
	this.clear(navigator.origin[0], navigator.origin[1], this.width / navigator.scale, this.height / navigator.scale)
 
	
	this.ctx.save();
	this.ctx.translate(this.navigator.panning[0], this.navigator.panning[1]);
	new debugDraw(this.ctx, world, this.scale, this.navigator.scale);
	this.ctx.restore();
	
	
}
