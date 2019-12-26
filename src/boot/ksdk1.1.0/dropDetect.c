typedef enum states = {
	init,
	standby,
	to_active,
	active,
	to_standby,
	deployed,
} ddState_t;

//
//	init	->	to_standby	->	standby
//			 *			 |
//			/ \			\ /
//			 |			 *
//			active		<-	to_active
//


ddState_t ddState = init;
void main(){
	while(1){
		switch(ddState)
		{
			case init:
				// initMMA
				// turn on transientDetection
				// ddState = to_standby
				break;
			case standby:
				// sleep for x milliseconds
				// if transientDetected
				// 	ddState = to_active
				break;
			case to_active:
				// turn on clocks
				// initialise servo
				// turn on drop detection
				// turn on tumble detection
				// ddState = active
				break;
			case active:
				// check for tumble
				// check for drop
				// if (tumble|drop)
				// 	spin servo
				// 	ddState = deployed
				// if !transientDetected
				// 	ddState = to_standby
				break;
			case to_standby:
				// disable drop detection
				// disable tumble detection
				// turn off servo
				// turn off clocks
				// ddState = standby
				break;
		}
	}
}



