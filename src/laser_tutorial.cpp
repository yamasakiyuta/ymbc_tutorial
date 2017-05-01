#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>

#include <stdio.h>
#include <math.h>
#include <ypspur.h>

int gIsShuttingDown;

void sec_sleep(double t){
    usleep(t*1000000);
}

void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}

int check_arg(int argc,char **argv){
  if( argc < 2 ) {
    fprintf(stderr, "ERORR: missing device operand\n");
    fprintf(stderr, "USAGE: %s <device>\n", argv[0]);
    fprintf(stderr, " e.g.: %s /dev/ttyACM0\n", argv[0]);
    return 1;
  }
  return 0;
}

int init_gnuplot(FILE *fp){
  if(fp == NULL) {
    fprintf(stderr, "ERROR: popen\n");
    return 1;
  }
  fprintf(stdout, "pipe opened\n");

  fputs("set terminal x11\n", fp);   /* drawing destination */
  fputs("set grid\n", fp);  /* draw grid */
  fputs("set mouse\n", fp);  /* use mouse */
  fputs("set xlabel \"y [m]\"\n", fp);  /* label of x-axis */
  fputs("set ylabel \"x [m]\"\n", fp);  /* label of y-axis */
  fputs("set xrange [6:-6]\n", fp);  /* range of x-axis */
  fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  fputs("set size ratio -1\n", fp);  /* aspect ratio */
  fputs("unset key\n", fp);  /* hide graph legends */
  //fputs("show yrange\n", fp);  /* hide graph legends */
  return 0;
}

int init_spur(void){
    // Windows環境で標準出力がバッファリングされないように設定
    setvbuf( stdout, 0, _IONBF, 0 );

    // 初期化
    if(Spur_init(  ) < 0 )
    {
        fprintf(stderr, "ERROR : cannot open spur.\n");
        return 1;
    }

    Spur_set_vel( 0.2 );
    Spur_set_accel( 0.2 );
    Spur_set_angvel( M_PI / 2.0 );
    Spur_set_angaccel( M_PI / 1.0 );

    Spur_set_pos_GL( 0, 0, 0 );

    return 0;
}

int main(int argc, char **argv) {
  /* variables */
  int ret;
  S2Port   *urg_port;   /* port */
  S2Sdd_t   urg_buff;   /* buffer */
  S2Scan_t *urg_data;   /* pointer to buffer */
  S2Param_t urg_param;  /* parameter */
  double x_GL, y_GL, theta_GL;
  double robot_radius = 0.35;
  int phase = 1;

  /* set Ctrl-c function */
  signal(SIGINT, ctrl_c);

  /* check argument */
  if(check_arg(argc,argv))return 1;

  /* open port */
  urg_port = Scip2_Open(argv[1], B115200);
  if(urg_port == 0) {
    fprintf(stderr, "ERORR: cannot open %s\n", argv[1]);
    return 1;
  }
  fprintf(stdout, "port opened\n");

  /* init buffer */
  S2Sdd_Init(&urg_buff);

  /* get paramerter */
  Scip2CMD_PP(urg_port, &urg_param);

  /* scan start */
  // Scip2CMD_StartMS(urg_port, 44, 725, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE);
  ret = Scip2CMD_StartMS(urg_port, urg_param.step_min, urg_param.step_max, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE );
  if(ret == 0) {
    fprintf(stderr, "ERROR: Scip2CMD_StartMS\n");
    return 1;
  }
  fprintf(stdout, "scan started\n");

  /* open pipe to gnuplot */
  /*FILE *fp = popen("gnuplot -noraise", "w");
  if(init_gnuplot(fp))return 1;*/

  /* init spur and set param */
  //if(init_spur())return -1;

  /* main loop */
  gIsShuttingDown = 0;
  while(!gIsShuttingDown) {
    int i;
    double x_FS, y_FS, rad_FS;
    double dist;
    double min_dist=100;

    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);

    if(ret > 0) {
      //fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        rad_FS = (2 * M_PI / urg_param.step_resolution) * 
                    (i + urg_param.step_min - urg_param.step_front);
        x_FS = urg_data->data[i] * cos(rad_FS) / 1000.0;
        y_FS = urg_data->data[i] * sin(rad_FS) / 1000.0;
        //fprintf(fp, "%f %f\n", y_FS, x_FS);

        dist = sqrt(x_FS*x_FS + y_FS*y_FS);
        //printf("dist = %g\n",dist);
        if(dist<min_dist){
            min_dist = dist;
        }
      }
      if(min_dist<robot_radius){
          printf("[CAUTION!] min_dist = %g\n",min_dist);
          //Spur_stop();
          sec_sleep(4);
      }
      else{
          printf("[safe] min_dist = %g\n",min_dist);
          //Spur_vel(0.2,0);
          switch(phase){
              case 1:
                  printf("aproaching wall\n");
                  break;
              case 2:
                  printf("run along wall\n");
                  break;
              case 3:
                  printf("pole\n");
                  break;
              case 4:
                  printf("garage\n");
                  break;
              default:
                  printf("others\n");
                  break;
          }

      }
      //fputs("e\n", fp);

      /* unlock buffer */
      S2Sdd_End(&urg_buff);

      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      usleep(90000);
    } else if(ret == -1) {
      fprintf(stderr, "ERROR: S2Sdd_Begin\n");
      break;
    } else {
      usleep(100);
    }

  }

  /* close pipe */
  //pclose(fp);
  fprintf(stdout, "pipe closed\n");

  /* scan stop */
  ret = Scip2CMD_StopMS(urg_port, &urg_buff);
  if(ret == 0) {
    fprintf(stderr, "ERROR: Scip2CMD_StopMS\n");
    return 1;
  }
  fprintf(stdout, "scan stopped\n");

  /* destruct buffer */
  S2Sdd_Dest(&urg_buff);

  /* close port */
  Scip2_Close(urg_port);
  fprintf(stdout, "port closed\n");
}

