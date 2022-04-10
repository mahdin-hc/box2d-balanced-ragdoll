function balance(loadedScene){

	var Bodies = loadedScene.bodies;

	function move_rotation(body, angle, amount = 1, isActive){
		if(isActive){
			body.SetAngularVelocity(-(body.GetAngle() + angle) * amount);
		}
	}

	var f = 10
	var is_active = true; // is balance enable?


	function update(time) {
		//make a smooth fixed body rotation
		move_rotation(Bodies[9], 0, f, is_active); // left up leg
		move_rotation(Bodies[8], 0, f, is_active); // right up leg
		move_rotation(Bodies[6], 0, f, is_active); // right low leg
		move_rotation(Bodies[7], 0, f, is_active); // left low leg
		move_rotation(Bodies[0], 0, f, is_active); // chest
		
		requestAnimationFrame(update);
	}

	requestAnimationFrame(update); // update evrything inside update(...){...}


}