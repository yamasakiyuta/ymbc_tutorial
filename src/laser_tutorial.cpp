#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>

#include <stdio.h>
#include <math.h>
#include <ypspur.h>

//urg_data->size = 682

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

    Spur_set_vel( 0.4 );
    Spur_set_accel( 0.2 );
    Spur_set_angvel( 0.6 );
    Spur_set_angaccel( M_PI / 2.0 );

    Spur_set_pos_GL( 0, 0, 0 );

    return 0;
}

double distance(double *x, double *y, int i, int j){
    return sqrt(pow(x[i]-x[j],2.0) + pow(y[i]-y[j],2.0));
}

void calc_line(double x1, double y1, double x2, double y2){
    printf("x1:%g,y1:%g,x2:%g,y2:%g\n",x1, y1, x2, y2);     
    double mx,my;
    double dx = x1 - x2;
    double dy = y1 - y2;
    if(dx>0)dy=dy+(-1);
    double dtheta = atan2(fabs(dx),dy);
    mx = (x1 + x2)/2;
    my = (y1 + y2)/2;
    printf("x:%g,y:%g,theta:%g\n",mx, my, dtheta*180/M_PI);     
    Spur_line_GL(mx, my, dtheta+M_PI/2);
    while( !Spur_over_line_GL(mx, my, dtheta+M_PI/2) )
    //while( !Spur_near_pos_GL(mx, my, 0.2) )
    usleep( 5000 );
}

int main(int argc, char **argv) {
  /* variables */
  int ret;
  S2Port   *urg_port;   /* port */
  S2Sdd_t   urg_buff;   /* buffer */
  S2Scan_t *urg_data;   /* pointer to buffer */
  S2Param_t urg_param;  /* parameter */
  double x_GL, y_GL, theta_GL;
  double robot_radius = 0.2;
  int phase = 1;
  int pos_reset_once=1;
  double x_p1, y_p1, x_p2, y_p2;

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
  if(init_spur())return -1;

  Spur_spin_GL( M_PI/8 );
  while( !Spur_near_ang_GL(M_PI/8, M_PI/18.0) )
  usleep( 5000 );
  Spur_stop();
  
  /* main loop */
  gIsShuttingDown = 0;
  while(!gIsShuttingDown) {
    int i;
    double x_FS[700], y_FS[700], rad_FS[700];
    double x_GL[700], y_GL[700], rad_GL[700];
    double robot_x,robot_y,robot_theta;
    double dist=0;
    double min_dist=100;
    int min_dist_num=0;
    double cmd_vel_linear=0.2;
    double cmd_vel_angular=0;
    double dx_FS=0;
    double dy_FS=0;
    double wall_offset=0.65;
    double dtheta=0;
    double pole_dist=0;
    double min_pole_dist=100;
    int min_pole_dist_num=0;
    int j=0;
    double tmp;
    double garage_l_x, garage_l_y;
    double garage_r_x, garage_r_y;
    int garage_l_num, garage_r_num;
    int search_garage_flag=0;
    int no_wall_flag=1;
    double wall_dist_min=3.0;
    int edge_num=0;
    double dxdy_FS[700];
    double edge_x[4];
    double edge_y[4];
    double dist_l=10,dist_f=10,dist_r=10;

    Spur_get_pos_GL(&robot_x, &robot_y, &robot_theta);
    
    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);

    if(ret > 0) {
      //fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        rad_FS[i] = (2 * M_PI / urg_param.step_resolution) * 
                    (i + urg_param.step_min - urg_param.step_front);
        x_FS[i] = urg_data->data[i] * cos(rad_FS[i]) / 1000.0;
        y_FS[i] = urg_data->data[i] * sin(rad_FS[i]) / 1000.0;
        
        rad_GL[i] = rad_FS[i] + robot_theta;
        x_GL[i] = robot_x + urg_data->data[i] * cos(rad_GL[i]) / 1000.0;
        y_GL[i] = robot_y + urg_data->data[i] * sin(rad_GL[i]) / 1000.0;

        //printf("i:%d    rad:%g\n",i,rad_FS[i]*180/M_PI);

        //fprintf(fp, "%f %f\n", y_FS, x_FS);

        dist = sqrt(x_FS[i]*x_FS[i] + y_FS[i]*y_FS[i]);
        //printf("dist = %g\n",dist);
        if(dist<min_dist){
            if(phase==8){
                if(i>119 && i<511){
            min_dist = dist;
            min_dist_num = i;
                }
            }
            else{
            min_dist = dist;
            min_dist_num = i;
            }

        }
            //obstacle direction
            if(i>=0 && i<119 && dist<dist_r){
                dist_r = dist;
            }
            else if(i>=119 && i<511 && dist<dist_f){
                dist_f = dist;
            }
            else if(i>=511 && i<600 && dist<dist_l){
                dist_l = dist;
            }
      }
                //printf("right:%g\n",dist_r);
                //printf("front:%g\n",dist_f);
                //printf("left:%g\n",dist_l);
      if(min_dist<robot_radius){
          printf("[CAUTION!] min_dist = %g\n",min_dist);
          //Spur_stop();
          //sec_sleep(4);
      }
      else{
          
          switch(phase){
              case 1:
                  
                  printf("aproaching wall\n dtheta = %g\n",dtheta);
          //printf("%g,%g\n",y_FS[600], y_FS[400]);
          dx_FS = x_FS[300]-x_FS[380];
          dy_FS = y_FS[300]-y_FS[380];
          dtheta = atan2(dx_FS,dy_FS);
          //printf("[safe] min_dist = %g\n",min_dist);
                  if(fabs(dx_FS)<0.1){
                      printf("straight\n");
                      cmd_vel_linear = 0.2;
                      cmd_vel_angular = 0.0;
                      if(x_FS[341]<wall_offset){
                          printf("phase 1 success\n");
                          cmd_vel_linear = 0.0;
                          cmd_vel_angular = 0.0;
                          Spur_stop();
                          phase = 2;
                      }
                  }
                  else{
                      printf("search wall\n");
                      if(x_FS[341]<3.0){
                          cmd_vel_linear=0.0;
                          cmd_vel_angular=0.2*dx_FS/fabs(dx_FS);
                      }
                      else{
                          cmd_vel_linear=0.0;
                          cmd_vel_angular=-0.1;
                      }
                  }
                  break;
              case 2:
                  printf("turn right\n");
                  Spur_set_pos_GL(0,0,0);
                  Spur_spin_GL( -M_PI/2 );
                  while( !Spur_near_ang_GL(-M_PI/2, M_PI/18.0) )
                  usleep( 5000 );
                  phase = 3;
                    Spur_set_pos_GL(0,0,0);

                  break;
              case 3:
                  printf("run along wall\n");
                  if( !Spur_over_line_GL(2.7, 0.0, 0.0) ){
                      if(fabs(y_FS[500]-y_FS[650])<0.05){
                        cmd_vel_linear=0.1;
                        cmd_vel_angular=0.0;
                      }
                      else{
                        cmd_vel_linear=0.0;
                        cmd_vel_angular=0.3*(y_FS[500]-y_FS[650])/fabs(y_FS[500]-y_FS[650]);
                      }
                  }
                  else{
                      phase = 4;
                      Spur_set_pos_GL(0,0,0);
                  }
                  break;
              case 4:
                  printf("turn right\n");
                  Spur_spin_GL( -M_PI/2 - M_PI/180*10 );
                  while( !Spur_near_ang_GL(-M_PI/2 - M_PI/180*10, M_PI/18.0) )
                  usleep( 5000 );
                  Spur_stop();
                  phase = 5;
                  Spur_set_pos_GL(0,0,0);

                  break;
              case 5:
                  printf("pole\n");
                  Spur_line_GL( 1.5, 0.0, 0.0 );
                  while( !Spur_over_line_GL(1.5, 0.0, 0.0) )
                  usleep( 5000 );
                  Spur_stop();
                  phase = 6;

                  break;
              case 6:
                  printf("turn right\n");
                  Spur_set_pos_GL(0,0,0);
                  Spur_spin_GL( -M_PI/2 );
                  while( !Spur_near_ang_GL(-M_PI/2, M_PI/18.0) )
                  usleep( 5000 );
                  Spur_stop();
                  phase = 7;

                  break;
              case 7:
                  printf("gate\n");
                  cmd_vel_linear=0;
                  cmd_vel_angular=-0.1;
                  for(j=0;j<urg_data->size;j++){
                      pole_dist  = distance(x_FS,y_FS,min_dist_num,j);
                      if(pole_dist>0.6 && pole_dist<0.85 && y_FS[j]<0){
                        x_p1=x_FS[min_dist_num];
                        y_p1=y_FS[min_dist_num];
                        x_p2=x_FS[j];
                        y_p2=y_FS[j];
                        if(y_p1<y_p2){
                            tmp = y_p1;
                            y_p1 = y_p2;
                            y_p2 = tmp;
                        }
                        //if(fabs(x_p1+x_p2)/2<0.1 && fabs(y_p1 + y_p2)/2<0.1){
                        if(fabs(x_p1+x_p2)/2<0.1 ){
                            phase = 8;
                            Spur_stop();
                            Spur_set_pos_GL(0,0,0);
                            break;
                        }
                        else if(fabs(y_p1 + y_p2)/2 >= 0.1){
                        printf("pole_dist = %g\n",pole_dist);
                            cmd_vel_linear = 0;
                            cmd_vel_angular = 0.2*(y_p1 + y_p2)/fabs(y_p1 + y_p2);
                            break;
                        }
                        else if(fabs(y_p1 + y_p2)/2<0.1){
                        printf("pole_dist = %g\n",pole_dist);
                            cmd_vel_linear = 0.1;
                            cmd_vel_angular = 0.0;
                            break;
                        }
                      }
                  }

                  break;
              case 8:
                        if(dist_f<0.3 && dist_l<0.3 && dist_r<0.3 ){
                            phase = 9;
                            Spur_stop();
                            Spur_set_pos_GL(0,0,0);
                            break;
                        }
                  for(j=119;j<511;j++){
                      pole_dist  = distance(x_FS,y_FS,min_dist_num,j);
                      if(pole_dist>0.45 && pole_dist<0.48 && fabs(y_FS[min_dist_num]-y_FS[j])>0.3){
                        x_p1=x_FS[min_dist_num];
                        y_p1=y_FS[min_dist_num];
                        x_p2=x_FS[j];
                        y_p2=y_FS[j];
                        if(y_p1<y_p2){
                            tmp = y_p1;
                            y_p1 = y_p2;
                            y_p2 = tmp;
                        }
                        //if(fabs(x_p1+x_p2)/2<0.1 && fabs(y_p1 + y_p2)/2<0.1){
                        if(fabs(x_p1+x_p2)/2<0.1 || dist_f<0.2){
                            phase = 9;
                            Spur_stop();
                            Spur_set_pos_GL(0,0,0);
                            break;
                        }
                        else if(fabs(y_p1 + y_p2)/2 >= 0.1){
                        printf("pole_dist = %g\n",pole_dist);
                            cmd_vel_linear = 0;
                            cmd_vel_angular = 0.2*(y_p1 + y_p2)/fabs(y_p1 + y_p2);
                            break;
                        }
                        else if(fabs(y_p1 + y_p2)/2<0.1){
                        printf("pole_dist = %g\n",pole_dist);
                            cmd_vel_linear = 0.1;
                            cmd_vel_angular = 0.0;
                            break;
                        }
                      }
                  }
                  /*for(j=500;j>=100;j--){
                      if(search_garage_flag==0){
                          if(x_GL[j]>0.3 && x_GL[j]<1.2 ){
                              if(urg_data->data[j]/1000<wall_dist_min){
                                  wall_dist_min=urg_data->data[j]/1000;
                                  garage_l_num = j;
                                  printf("wall_dist_min:%g\n",wall_dist_min);
                              }
                              garage_l_x = x_FS[j];
                              garage_l_y = y_FS[j];
                              printf("l_x:%g, l_y:%g\n",garage_l_x, garage_l_y);
                              if(j==1){
                                  search_garage_flag=1;
                                  j=500;
                                  continue;
                              }
                          }
                      }
                      else{
                          if(fabs(y_FS[j]-garage_l_y)>0.3 && fabs(y_FS[j]-garage_l_y)<0.6 && fabs(x_FS[j]-garage_l_x)<0.3){
                              garage_r_x = x_FS[j];
                              garage_r_y = y_FS[j];
                              printf("r_x:%g, r_y:%g\n",garage_r_x, garage_r_y);
                              search_garage_flag=0;
                              no_wall_flag=0;
                              printf("wall FOUND\n");
                              break;
                          }
                          else{
                              no_wall_flag=1;
                          }
                      }
                  }
                  if(no_wall_flag==1){
                      printf("no_wall\n");
                      cmd_vel_linear=0;
                      cmd_vel_angular=-0.1;
                  }
                  else{
                      if(fabs(garage_l_y +garage_r_y)/2>0.1){
                          cmd_vel_linear=0;
                          cmd_vel_angular=0.2 * (garage_l_y +garage_r_y)/fabs(garage_l_y +garage_r_y);
                      }
                      else if(fabs(garage_l_y +garage_r_y)/2<=0.1){
                          cmd_vel_linear=0.1;
                          cmd_vel_angular=0;
                      }
                      else if(urg_data->data[100]<0.3 && urg_data->data[341]<0.3 && urg_data->data[600]<0.3){
                          phase = 10;
                          Spur_stop();
                      }
                      else{
                          cmd_vel_linear=0;
                          cmd_vel_angular=0.2;
                      }
                  }*/
                  /*printf("%g\n",min_dist);
                  for(j=min_dist_num;j>=169;j--){
                      if(sqrt(x_FS[j]*x_FS[j] + y_FS[j]*y_FS[j])<wall_dist_min){
                          wall_dist_min = sqrt(x_FS[j]*x_FS[j] + y_FS[j]*y_FS[j]);
                          garage_r_num=j;
                      }
                  }*/
                  /*for(j=511;j>119;j-=10){
                      if(x_FS[j] < 2.0 && x_FS[j]>0){
                          dxdy_FS[j]=(x_FS[j]-x_FS[j-10])/fabs(y_FS[j]-y_FS[j-10]);
                          printf("dxdy:%g\n",dxdy_FS[j]);
                                  if((dxdy_FS[j]*dxdy_FS[j-1])<0){
                                      edge_x[edge_num]=x_FS[j];
                                      edge_y[edge_num]=y_FS[j];
                                      garage_l_x=x_FS[j];
                                      garage_r_x=x_FS[j];
                                      printf("%g , %d\n",x_FS[j],j);
                                      edge_num++;
                                  }
                      }
                  }
                  printf("edge:%d\n",edge_num);
                  if(edge_num>=2){
                  for(j=0;j<=edge_num;j++){
                      if(edge_x[j]<garage_l_x){
                          garage_l_x=edge_x[j];
                          garage_l_y=edge_y[j];
                          garage_l_num=j;
                      }
                  }
                  for(j=0;j<=edge_num;j++){
                      if(j==garage_l_num){
                          continue;
                      }

                      if(edge_x[j]<garage_r_x && fabs(edge_y[j]-garage_l_y)>0.2){
                          garage_r_x=edge_x[j];
                          garage_r_y=edge_y[j];
                          garage_r_num=j;
                      }
                  }
                        x_p1=garage_l_x;
                        y_p1=garage_l_y;
                        x_p2=garage_r_x;
                        y_p2=garage_r_y;
                        if(y_p1<y_p2){
                            tmp = y_p1;
                            y_p1 = y_p2;
                            y_p2 = tmp;
                        }
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                        if((x_p1+x_p2)/2<0.1){
                            phase = 9;
                            break;
                        }
                        if(fabs(y_p1 + y_p2)/2 > 0.1){
                            cmd_vel_linear = 0;
                            cmd_vel_angular = 0.2*(y_p1 + y_p2)/fabs(y_p1 + y_p2);
                            printf("pole_dist = %g\n",pole_dist);
                        //printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                        else{
                            cmd_vel_linear = 0.1;
                            cmd_vel_angular = 0.0;
                            printf("pole_dist = %g\n",pole_dist);
                        //printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                  }*/
/*
                  printf("%g\n",wall_dist_min);
                      pole_dist = distance(x_FS,y_FS,min_dist_num,garage_r_num);
                        printf("%g\n",pole_dist);
                      if(pole_dist>0.0 && pole_dist<0.6 && fabs(x_FS[min_dist_num]-x_FS[garage_r_num])<0.3 ){
                          printf("found\n");
                        x_p1=x_FS[min_dist_num];
                        y_p1=y_FS[min_dist_num];
                        x_p2=x_FS[garage_r_num];
                        y_p2=y_FS[garage_r_num];
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                        if((x_p1+x_p2)/2<0.1){
                            phase = 9;
                            break;
                        }
                        if(y_p1<y_p2){
                            tmp = y_p1;
                            y_p1 = y_p2;
                            y_p2 = tmp;
                        }
                        if(fabs(y_p1 + y_p2)/2 > 0.1){
                            cmd_vel_linear = 0;
                            cmd_vel_angular = 0.2*(y_p1 + y_p2)/fabs(y_p1 + y_p2);
                            printf("pole_dist = %g\n",pole_dist);
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                        else{
                            cmd_vel_linear = 0.1;
                            cmd_vel_angular = 0.0;
                            printf("pole_dist = %g\n",pole_dist);
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                      }
                      else{
                          printf("searching\n");
                          cmd_vel_linear=0;
                          cmd_vel_angular=-0.1;
                      }*/
/*
                  for(j=511;j>=169;j--){
                      pole_dist = distance(x_FS,y_FS,min_dist_num,j);
                      if(pole_dist>0.4 && pole_dist<0.6 && fabs(x_FS[min_dist_num]-x_FS[j])<0.3 ){
                          printf("found\n");
                        x_p1=x_FS[min_dist_num];
                        y_p1=y_FS[min_dist_num];
                        x_p2=x_FS[j];
                        y_p2=y_FS[j];
                        Spur_set_pos_GL(0,0,0);
                        Spur_line_GL((x_p1+x_p2)/2,(y_p1+y_p2)/2,0);
                        while( !Spur_over_line_GL((x_p1+x_p2)/2,(y_p1+y_p2)/2,0) )
                        usleep( 5000 );
                        Spur_stop();
                        phase=10;
                        break;
                        if((x_p1+x_p2)/2<0.1){
                            phase = 9;
                            break;
                        }
                        if(y_p1<y_p2){
                            tmp = y_p1;
                            y_p1 = y_p2;
                            y_p2 = tmp;
                        }
                        if(fabs(y_p1 + y_p2)/2 > 0.1){
                            cmd_vel_linear = 0;
                            cmd_vel_angular = 0.2*(y_p1 + y_p2)/fabs(y_p1 + y_p2);
                            printf("pole_dist = %g\n",pole_dist);
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                        else{
                            cmd_vel_linear = 0.1;
                            cmd_vel_angular = 0.0;
                            printf("pole_dist = %g\n",pole_dist);
                        printf("x1:%g, y1:%g, x2:%g, y2:%g\n",x_p1, y_p1, x_p2, y_p2);
                            break;
                        }
                      }
                      else{
                          printf("searching\n");
                          cmd_vel_linear=0;
                          cmd_vel_angular=-0.1;
                      }
                  }
*/
                  break;
              case 9:
                  printf("finish\n");
                  phase =10;
                  cmd_vel_linear=0;
                  cmd_vel_angular=0;
                  break;
              default:
                  //printf("others\n");
                  Spur_stop();
                  cmd_vel_linear = 0;
                  cmd_vel_angular = 0;
                  break;
          }
          //printf("linear:%g    angular:%g\n",cmd_vel_linear, cmd_vel_angular);
          //if(phase==8 && dist_r<0.5 || dist_l<0.5){
          //  cmd_vel_angular += (dist_r - dist_l)*0.01;
          //}
          if(phase==10)break;

          Spur_vel(cmd_vel_linear,cmd_vel_angular);

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

