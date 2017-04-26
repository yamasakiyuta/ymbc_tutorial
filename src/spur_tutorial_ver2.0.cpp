#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

#ifdef __WIN32
#	include <windows.h>
#endif

int main( int argc, char *argv[] )
{
	double x, y, theta;
	
	// Windows環境で標準出力がバッファリングされないように設定
	setvbuf( stdout, 0, _IONBF, 0 );
	
	// 初期化
	if(	Spur_init(  ) < 0 )
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return -1;
	}

	Spur_set_vel( 0.8 );
	Spur_set_accel( 0.8 );
	Spur_set_angvel( 2.0*M_PI / 1.0 );
	Spur_set_angaccel( 3.0*M_PI / 1.0 );

	Spur_set_pos_GL( 0, 0, 0 );

	//PoleA
	Spur_circle_GL( 1.0, 0.0, 0.6 );
	printf( "spin A1\n" );
	while( !Spur_over_line_GL( 1.0, 0.0, 0.0 ) )
		usleep( 50000 );
	printf( "spin A2\n" );
	while( !Spur_over_line_GL( 1.0, 0.0, M_PI ) )
		usleep( 50000 );
	printf( "spin A3\n" );
	while( !Spur_over_line_GL( 1.0, 0.0, 0.0 ) )
		usleep( 50000 );

    //PoleB
	printf( "spin B1\n" );
	Spur_circle_GL( 3.0, -0.30, -0.6 );
	while( !Spur_over_line_GL( 3.0, 0.0, 0.0 ) )
		usleep( 50000 );
	printf( "spin B2\n" );
	while( !Spur_over_line_GL( 3.0, 0.0, M_PI ) )
		usleep( 50000 );
/*	printf( "spin B3\n" );
	while( !Spur_over_line_GL( 3.0, 0.0 - 0.005, M_PI/2.0 ) )
		usleep( 50000 );
*/
//Gate
	printf( "spin GATE1\n" );
    Spur_line_GL( 2.2, 1.6, M_PI/2.0 );
	while( !Spur_over_line_GL( 2.2 , 1.6 - 0.005, M_PI/2.0 ) )
		usleep( 50000 );
/*
    Spur_line_GL( 1.4, 2.2, M_PI );
	while( !Spur_over_line_GL( 1.4 + 0.005 , 2.2 , M_PI ) )
		usleep( 50000 );
*/
    Spur_line_GL( -0.2, 0.3, -3*M_PI/4.0 );
	while( !Spur_over_line_GL( -0.2, 0.3, M_PI ) )
		usleep( 50000 );

	// 回転
	printf( "spin\n" );
	Spur_spin_GL( 0.0 );
	while( !Spur_near_ang_GL( 0.0 - 6*M_PI/18, M_PI / 18.0 ) )
		usleep( 5000 );

	// ちょっと加速
	//Spur_set_vel( 0.3 );
	//Spur_set_accel( 1.0 );
	//Spur_set_angvel( M_PI  );
	//Spur_set_angaccel( M_PI );
	

	Spur_stop(  );
	usleep( 4000000 );
	Spur_free(  );
	printf( "Hit Ctrl-C to exit.\n" );
	while( 1 )
	{
		Spur_get_pos_GL( &x, &y, &theta );
		printf( "%f %f %f\n", x, y, theta * 180.0 / M_PI );
		usleep( 1000000 );
	}
	
	return 0;
}
