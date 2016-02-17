/************************
Global Variables
*************************/
bool enc1AState, enc1BState, enc2AState, enc2BState;	//Encoder channel states
const int enc1A = 25;	//Pins for encoders
const int enc1B = 26;	//Pins for encoders
const int enc2A = 28;	//Pins for encoders
const int enc2B = 29;	//Pins for encoders

long count1, count2;

/************************
	Setup...
*************************/
void setup()
{
	count1 = 0;
	count2 = 0;
	
	pinMode(enc1A, INPUT_PULLUP);
	pinMode(enc1B, INPUT_PULLUP);
	enc1AState = digitalRead(enc1A);
	enc1BState = digitalRead(enc1B);
	attachInterrupt(digitalPinToInterrupt(enc1A), enc1A_changed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc1B), enc1B_changed, CHANGE);
	
	pinMode(enc2A, INPUT_PULLUP);
	pinMode(enc2B, INPUT_PULLUP);
	enc2AState = digitalRead(enc2A);
	enc2BState = digitalRead(enc2B);
	attachInterrupt(digitalPinToInterrupt(enc2A), enc2A_changed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc2B), enc2B_changed, CHANGE);
	
}

/************************
	Loop...
*************************/
void loop()
{
	motorPin1 = 
	
	
	while(1)
	{
		
	}
}

/************************
Additional Functions
*************************/
float pid(float state, float stateCommand, float Ts, float tau, float kp, float ki, float kd, float integratorThresh, float controlThresh, bool flag)
{
	static float integrator, differentiator, error_d1;
	
	if(flag)
	{
		integrator = 0.0;
		differentiator = 0.0;
		error_d1 = 0.0;
	}
	
	float error = stateCommand - state;
	
	if(error < integratorThresh)
	{
		integrator = integrator + (Ts/2.0)*(error + error_d1);
	}
	else
	{
		integrator = 0.0;
	}
	
	differentiator = (2.0*tau - Ts)/(2.0*tau + Ts)*differentiator + 2.0/(2.0*tau + Ts)*(error - error_d1);
	
	return(saturate(kp*error + ki*integrator + kd*differentiator, controlThresh));
	
}

float saturate(float control, float controlThresh)
{
	if(control > controlThresh)
	{
		control = controlThresh;
	}
	else if(control < -controlThresh)
	{
		control = -controlThresh;
	}
	
	return(control);
}

void enc1A_changed()
{
    enc1AState = digitalRead(enc1A);
    if(enc1AState)  //A is HIGH
    {
        if(enc1BState)  //B is HIGH
        {
            count1--;
        }
        else
        {
            count1++;
        }
    }
    else
    {
        if(enc1BState)
        {
            count1++;
        }
        else
        {
            count1--;
        }
    }
}

void enc1B_changed()
{
    enc1BState = digitalRead(enc1B);

    if(enc1BState)  //B is HIGH
    {
        if(enc1AState)  //A is HIGH
        {
            count1++;
        }
        else  //A is LOW
        {
            count1--;
        }
    }
    else  //B is LOW
    {
        if(enc1AState)  //A is HIGH
        {
            count1--;
        }
        else  //A is LOW
        {
            count1++;
        }
    }
}


void enc2A_changed()
{
    enc2AState = digitalRead(enc2A);
    if(enc2AState)  //A is HIGH
    {
        if(enc2BState)  //B is HIGH
        {
            count2--;
        }
        else
        {
            count2++;
        }
    }
    else
    {
        if(enc2BState)
        {
            count2++;
        }
        else
        {
            count2--;
        }
    }
}

void enc2B_changed()
{
    enc2BState = digitalRead(enc2B);

    if(enc2BState)  //B is HIGH
    {
        if(enc2AState)  //A is HIGH
        {
            count2++;
        }
        else  //A is LOW
        {
            count2--;
        }
    }
    else  //B is LOW
    {
        if(enc2AState)  //A is HIGH
        {
            count2--;
        }
        else  //A is LOW
        {
            count2++;
        }
    }
}