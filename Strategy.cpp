#include "stdafx.h"
#include "Strategy.h"

#include <math.h>
 
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}

void Velocity ( Robot *robot, int vl, int vr );
void Angle ( Robot *robot, double desired_angle);
void Position( Robot *robot, double x, double y, double offset=0.0);
void goalie(Environment *env);
void goalie_angle(Robot* robot, double desired_angle);
bool set_goalie_right(Robot *robot);
void PredictBall ( Environment *env );
void general(Environment *env);
void penalty_goalie(Environment *env);
void penalty_attack(Environment *env);
void check_valid(Environment *env);
void for_attack(Environment *env);
void for_defense(Environment *env);
double distance(Vector3D v1, Vector3D v2);
bool set_def_orn(Robot* r, Ball ball);
void angleToVelocity(Robot* , double); 

const double PI = 3.1415923;

extern "C" STRATEGY_API void Create ( Environment *env )
{
    // allocate user data and assign to env->userData
    // eg. env->userData = ( void * ) new MyVariables ();
}

extern "C" STRATEGY_API void Destroy ( Environment *env )
{
    // free any user data created in Create ( Environment * )

    // eg. if ( env->userData != NULL ) delete ( MyVariables * ) env->userData;
}

//int i = 0;
extern "C" STRATEGY_API void Strategy ( Environment *env )
{
    PredictBall(env);
    switch(env->gameState)
    {
        case FREE_BALL:     break;
        case PENALTY_KICK:  //if(env->whosBall == YELLOW_BALL)
                                penalty_attack(env);
                            //else
                                penalty_goalie(env);
                            break;
        default:            general(env);break;
    }
    if(env->currentBall.pos.x > 50.0)
        for_attack(env);
	else
		for_defense(env);
	//check_valid(env);
	/*if(i>60)
		angleToVelocity(&env->home[1], -90.0);
	else
		Velocity(&env->home[1], 50, 50);
	i++;*/

}

double distance(Vector3D v1, Vector3D v2)
{
	return sqrt(pow(v2.x-v1.x, 2) + pow(v2.y-v1.y, 2) + pow(v2.z-v1.z, 2));
}
void for_attack(Environment* env)
{   
    
	Robot *sweeper, *non_sweeper, *active_attack, *passive_attack;
	if(distance(env->home[3].pos, env->currentBall.pos) < distance(env->home[4].pos, env->currentBall.pos))
	{
		active_attack = &env->home[3];
		passive_attack = &env->home[4];
	}
	else
	{
		active_attack = &env->home[4];
		passive_attack = &env->home[3];
	}
	Position(passive_attack, env->predictedBall.pos.x-20, (FTOP-FBOT)/2, 5);

    if(env->home[1].pos.x < env->home[2].pos.x)
    {
        sweeper = &env->home[1];
        non_sweeper = &env->home[2];
    }
    else
    {
        sweeper = &env->home[2];
        non_sweeper = &env->home[1];
    }
	Position(sweeper, PRIGHTX, env->predictedBall.pos.y, 5);
	Position(non_sweeper, HALFLINE, env->predictedBall.pos.y, 5);        
}
bool set_def_orn(Robot* r, Ball ball)
{
	if((r->pos.x>ball.pos.x) || ((r->pos.x + 10 > ball.pos.x) && !(r->rotation < 2 && r->rotation > -2)))
	{
		Position(r, ball.pos.x-10, (ball.pos.y + (FTOP-FBOT)/2)/2);
		return false;
	}
	else if (!(r->rotation < 2 && r->rotation > -2))
	{
		Angle(r, 0.0);
		return false;
	}
	return true;
}
void for_defense(Environment* env)
{

	Position(&env->home[3], HALFLINE, env->home[3].pos.y, 5);
	Position(&env->home[4], HALFLINE, env->home[4].pos.y, 5);
	Robot *active_def, *passive_def;

	active_def = (&env->home[1].pos.x > &env->home[2].pos.x)?&env->home[2]:&env->home[1];
	passive_def = (active_def == &env->home[2])?&env->home[1]:&env->home[2];

	set_def_orn(passive_def, env->predictedBall);
	Position(active_def, PRIGHTX, env->predictedBall.pos.y, 2);


}

const double Kp = 0.5, Kd = -0.256446;
double p_theta_e = 0;
void angleToVelocity(Robot* r, double desired_angle)
{
	// +ve theta_e = anticlockwise rotation
	double theta_e = desired_angle - r->rotation;
	double wheelVelocity = Kp*theta_e + Kd*(p_theta_e - theta_e);
	p_theta_e = theta_e;
	Velocity(r, -wheelVelocity, wheelVelocity);
}

void check_valid(Environment *env)
{
    // three in penalty area, defenders only
    if(env->home[3].pos.x < PRIGHTX + 5 && env->home[3].pos.y < PTOPY + 5 && env->home[3].pos.y > PBOTY - 5 )
    {
        if(&env->home[3].velocityLeft == 0 &env->home[3].velocityRight == 0)
            Velocity(&env->home[3], -50, -50);
        else
            Velocity(&env->home[3], 0, 0);
    }
    if(env->home[4].pos.x < PRIGHTX + 5 && env->home[4].pos.y < PTOPY + 5 && env->home[4].pos.y > PBOTY - 5 )
    {
        if(&env->home[4].velocityLeft == 0 &env->home[4].velocityRight == 0)
            Velocity(&env->home[4], -50, -50);
        else
            Velocity(&env->home[4], 0, 0);
    }
    
    // one in goal area, goalie only
    if(env->home[2].pos.x < GARIGHTX + 1 && env->home[2].pos.y < GATOPY + 1 && env->home[2].pos.y > GABOTY - 1)
    {
        if(&env->home[4].velocityLeft == 0 &env->home[4].velocityRight == 0)
            Velocity(&env->home[2], -50, -50);
        else
            Velocity(&env->home[2], 0, 0);
    }
    if(env->home[1].pos.x < GARIGHTX + 1 && env->home[1].pos.y < GATOPY + 1 && env->home[1].pos.y > GABOTY - 1)
    {
        if(&env->home[4].velocityLeft == 0 &env->home[4].velocityRight == 0)
            Velocity(&env->home[1], -50, -50);
        else
            Velocity(&env->home[1], 0, 0);
    }
    // opponent goal area, one attacker only
    if(env->home[3].pos.x > FRIGHTX - 5.9055 - 1 && env->home[3].pos.y < GATOPY + 1 && env->home[3].pos.y > GABOTY - 1 && 
        env->home[4].pos.x > FRIGHTX - 5.9055 && env->home[4].pos.y < GATOPY && env->home[4].pos.y > GABOTY)
        Velocity(&env->home[3], 0, 0);
    if(env->home[4].pos.x > FRIGHTX - 5.9055 - 1 && env->home[4].pos.y < GATOPY + 1 && env->home[4].pos.y > GABOTY - 1 && 
        env->home[3].pos.x > FRIGHTX - 5.9055 && env->home[3].pos.y < GATOPY && env->home[3].pos.y > GABOTY)
        Velocity(&env->home[4], 0, 0);
}
void general(Environment *env)
{
    //goalie
    goalie(env);
    
    //attacking

    Position(&env->home[3], env->predictedBall.pos.x, env->predictedBall.pos.y);
    Position(&env->home[4], env->predictedBall.pos.x, env->predictedBall.pos.y);
    Position(&env->home[2], env->predictedBall.pos.x, env->predictedBall.pos.y);
    Position(&env->home[1], env->predictedBall.pos.x, env->predictedBall.pos.y);
    
    //defence
}
void penalty_attack( Environment *env)
{
    Velocity(&env->home[4], 125, 125);
}

void penalty_goalie( Environment *env)
{
    int ball_y = (env->currentBall.pos.y<GBOTY)?GBOTY:((env->currentBall.pos.y>GTOPY)?GTOPY:env->currentBall.pos.y);
    if(ball_y > env->home[0].pos.y+0.5)
        Velocity(&env->home[0], 125, 125);
    else if(ball_y < env->home[0].pos.y-0.5)
        Velocity(&env->home[0], -125, -125);
    else
        Velocity(&env->home[0], 0, 0);
}
void Velocity ( Robot *robot, int vl, int vr )
{
    robot->velocityLeft = vl;
    robot->velocityRight = vr;
}

void PredictBall ( Environment *env )
{
    double dx = env->currentBall.pos.x - env->lastBall.pos.x;
    double dy = env->currentBall.pos.y - env->lastBall.pos.y;
    env->predictedBall.pos.x = env->currentBall.pos.x + dx;
    env->predictedBall.pos.y = env->currentBall.pos.y + dy;
}

void Position( Robot *robot, double x, double y, double offset)
{
    double diff_y = y - robot->pos.y;
    double diff_x = x - robot->pos.x;
    if(!(fabs(diff_x) < offset && fabs(diff_y)<offset))
	{
		double tanOfAngle = diff_y/diff_x;
		double reqAngle = atan(tanOfAngle)*180/PI;
		if(diff_y <0 && diff_x <0)
	        reqAngle -= 180;
	    if(diff_x <0 && diff_y >0)
			reqAngle +=180;
		Angle(robot, reqAngle);
	}
        
}

void Angle ( Robot *robot, double desired_angle)
{
    double theta_e;
    int vl, vr;
    short hard_turn = 0;
    theta_e = desired_angle - robot->rotation;
    
    while (theta_e > 180) theta_e -= 360;
    while (theta_e < -180) theta_e += 360;
    if(theta_e > 90 || theta_e < -90)
        hard_turn = 0;
    
    if(theta_e < -10)
    {
        vl = (pow(theta_e, 2)/(90.0*90.0)*125+15);
        vr = hard_turn*(pow(theta_e, 2)/(90.0*90.0)*50+5);
    }
    else if(theta_e > 10)
    {
        vr = (pow(theta_e, 2)/(90.0*90.0)*125+15);
        vl = hard_turn*(pow(theta_e, 2)/(90.0*90.0)*50+5);
    } 
    else
        vr = vl = 125;


    Velocity (robot, vl, vr);
    
}

void goalie_angle(Robot* robot, double desired_angle)
{
    double theta_e;
    int vl, vr;
    theta_e = desired_angle - robot->rotation;
    while (theta_e > 180) theta_e -= 360;
    while (theta_e < -180) theta_e += 360;
    if(theta_e < 0)
    {
        vl = (pow(theta_e, 2)/(90.0*90.0)*125+5);
        vr = -(pow(theta_e, 2)/(90.0*90.0)*125+5);
    }
    else if(theta_e > 0)
    {
        vr = (pow(theta_e, 2)/(90.0*90.0)*125+5);
        vl = -(pow(theta_e, 2)/(90.0*90.0)*125+5);
    } 
    else
        vr = vl = 0;
    Velocity (robot, vl, vr);
}

bool set_goalie_right(Robot *robot)
{
    if(robot->pos.x > 9 && robot->pos.x < 11)
    {
        if(robot->rotation > 88 && robot->rotation < 92)
            return true;
        else
            goalie_angle(robot, 90.0);
    }
    else//not the right x
    {
        if(robot->rotation > -2 && robot->rotation < 2)//right orientation
        {
            if(robot->pos.x < 9)    //go forward
                Velocity(robot, 80, 80);
            else if(robot->pos.x > 11) //go backward
                Velocity(robot, -80, -80);
        }
        else
            goalie_angle(robot, 0.0);
    }
    return false;

}

void goalie(Environment *env)
{
    if(set_goalie_right(&env->home[0]))
	{   
        int ball_y = (env->predictedBall.pos.y<GBOTY)?GBOTY:((env->predictedBall.pos.y>GTOPY)?GTOPY:env->predictedBall.pos.y);
        if(ball_y > env->home[0].pos.y + 1)
            Velocity(&env->home[0], 125, 125);
        else if(ball_y < env->home[0].pos.y - 1)
            Velocity(&env->home[0], -125, -125);
        else
            Velocity(&env->home[0], 0, 0);
    }
}