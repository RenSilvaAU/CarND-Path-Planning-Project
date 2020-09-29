# Path Generation
The path is created by creating a list of sparsely populated waypoints (30,60,90 metres) fitting a spline and then using a spline library to generate equaly spaced points in between. 

### This is how this is achived:

* First, I take the intial position of the car (if it is stationary), or the last two points of the last trajectory that was calculated.
* Then I add the three sparsely populated points(30,60,90 metres) to the list (obtaining a list with 4 or 5 points), that fit a spline
* Then I calculate points to be sent to the simulator every 0.02 seconds (the number of metres between them is calculated so that the vehicle moves at a given speed (which is stored in variable variable `ref_speed`))

### In addition, the following adjustments are made:
* `ref_speed` is initially set to zero, and increases gradualy by **0.224 MPH** (this minimises acceleration) until it reaches `max_limit` (which I set to **49.5 MPH**), or it is too close to the vehicle ahead
* if the vehicle is too close to the vehicle ahead, the speed will reduce by **0.224 MPH** until it is marginally slower than the vehicle ahead (from then on it starts to match the speed of the vehicle ahead) 
* once the vehicle finds itself behind a slower vehicle, it will attempt to overtake it. If it is safe, it will overtake the vehicle by the left, otherwise, it will try to undertake the vehicle by the right, if safe.
* if neither an overtake or undertake is safe (to be safe, there must be no vehicle in the target lane in the range of -25m to +60m), the vehicle stays in its lane, matching the speed of the vehicle ahead, until it is safe to overtake
* once the vehicle finds itself in the outside line, it will return to the middle lane as soon as it is safe to do so.
