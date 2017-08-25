/***********************************************
* checkElevatorOpenStatus
* param:  frame 360 scanDots
* return: true open
*         false close
************************************************/
extern bool startCheck = false;

bool checkElevatorOpenStatus(std::vector<scanDot> & frame) 
{
	static bool openStatus = false;
	int sumDist = 0, num = 0;
	static float averDist = 0.0;

	// select points between 335~25 degree
	std::vector<scanDot> elevatorDots;
	for (int i = 0; i < frame.size(); i++)
	{
		if (frame[i].angle < 25 || frame[i].angle > 335)
		{
			sumDist += frame[i].dist * cos(frame[i].angle*PI / 180);
			num++;
			elevatorDots.push_back(frame[i]);
		}
	}

	if (num > 1) // avoid the divident to be zero
	{
		if (startCheck)  // just compute averDist once when call for this function
		{
			averDist = sumDist / num;
			startCheck = false;
		}
		int goodDotsCount = 0;
		for (int i = 0; i < elevatorDots.size(); i++)
		{
			float distance = elevatorDots[i].dist * cos(elevatorDots[i].angle*PI / 180);
			if (distance > averDist - 100 && distance < averDist + 100)
				goodDotsCount++;
		}
		if (goodDotsCount < 0.3 * elevatorDots.size())
		{
			openStatus = true;
		}
		else if (goodDotsCount > 0.6 * elevatorDots.size())
		{
			openStatus = false;
		}
	}
	return openStatus;
}