/*
 * 	calculate the current coordinate based on locations and distances of/from ally beacons
 */
Coordinate calculateCoord() {
	// myBeacon2 is the start point, myBeacon1 is the end point
	int16_t dx, dy;
	float theta, beta, errpos, errneg;
	Coordinate pos, neg;
	dx = myBeacon1.coord.x_pos - myBeacon2.coord.x_pos;
	dy = myBeacon1.coord.y_pos - myBeacon2.coord.y_pos;
	beta = acos((dx * dx + dy * dy + myBeacon2.distance * myBeacon2.distance - myBeacon1.distance * myBeacon1.distance)
							/ ((float) 2 * myBeacon2.distance * sqrt(dx * dx + dy * dy)));
	theta = acosf((float) dx / (dx * dx + dy * dy));
	if (dy < 0)
		theta = 2 * M_PI - theta;

	// get the coordinates
	pos.x_pos = myBeacon2.coord.x_pos + myBeacon2.distance * cosf(theta + beta);
	pos.y_pos = myBeacon2.coord.y_pos + myBeacon2.distance * sinf(theta + beta);
	neg.x_pos = myBeacon2.coord.x_pos + myBeacon2.distance * cosf(theta - beta);
	neg.y_pos = myBeacon2.coord.y_pos + myBeacon2.distance * sinf(theta - beta);

	// test for the correct one
	errpos = powf(pos.x_pos - myBeacon3.coord.x_pos, 2)
			+ powf(pos.y_pos - myBeacon3.coord.y_pos, 2)
			- powf(myBeacon3.distance, 2);
	errneg = powf(neg.x_pos - myBeacon3.coord.x_pos, 2)
			+ powf(pos.y_pos - myBeacon3.coord.y_pos, 2)
			- powf(myBeacon3.distance, 2);

	// myBeacon2 is the start point, myBeacon1 is the end point
	int16_t dx, dy;
	float theta, beta, errpos, errneg, temp;
	Coordinate pos, neg;
	dx = myBeacon1.coord.x_pos - myBeacon2.coord.x_pos;
	dy = myBeacon2.coord.y_pos - myBeacon1.coord.y_pos;
	beta =
			acos(
					(dx * dx + dy * dy + myBeacon2.distance * myBeacon2.distance
							- myBeacon1.distance * myBeacon1.distance)
							/ ((float) 2 * myBeacon2.distance
									* sqrt(dx * dx + dy * dy)));
	theta = acosf((float) dx / (sqrt(dx * dx + dy * dy)));
	if (dy < 0)
		theta = 2 * M_PI - theta;

	// get the coordinates
	pos.x_pos = myBeacon2.coord.x_pos + cosf(theta + beta) * myBeacon2.distance;
	pos.y_pos = myBeacon2.coord.y_pos - sinf(theta + beta) * myBeacon2.distance;
	neg.x_pos = myBeacon2.coord.x_pos + cosf(theta - beta) * myBeacon2.distance;
	neg.y_pos = myBeacon2.coord.y_pos - sinf(theta - beta) * myBeacon2.distance;

	// test for the correct one
	errpos = sqrt(
			powf(pos.x_pos - myBeacon3.coord.x_pos, 2.0)
					+ powf(pos.y_pos - myBeacon3.coord.y_pos, 2.0))
			- myBeacon3.distance;
	errneg = sqrt(
			powf(neg.x_pos - myBeacon3.coord.x_pos, 2.0)
					+ powf(pos.y_pos - myBeacon3.coord.y_pos, 2.0))
			- myBeacon3.distance;

	if (errpos < errneg)
		return pos;
	return neg;
}
	

