//山彦セミナー　最終課題
//--------------------------------------------
//include
//--------------------------------------------
#include<cstdio> //stdio.hのc++版
#include<cmath> //math.hのc++版
#include<csignal> //signal.hのc++版
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/visualization/cloud_viewer.h> //cloudViewerクラスを使う
#include<pcl/ModelCoefficients.h>  
#include<pcl/io/pcd_io.h>  
#include<pcl/features/normal_3d.h>  
#include<pcl/filters/extract_indices.h>  
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/radius_outlier_removal.h> 
#include<pcl/kdtree/kdtree.h>  
#include<pcl/sample_consensus/method_types.h>  
//#include <pcl/sample_consensus/model_types.h>  
//#include <pcl/segmentation/sac_segmentation.h>  
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/common/centroid.h>
//RPP
#include<scip2awd.h> //ロボ研のURGドライバ(NewPCでインストール済み)
#include<ypspur.h>

const static int start_phase = 0;
const static int debug_laser = 0;
const static double leaf_size = 0.01; //1cm
const static double cluster_tolerance = 0.1; //10cm
const static int min_cluster_size = 5;
const static int max_cluster_size = 700;
const static double removal_radius = 0.05;
const static int removal_neighbors = 2;
const static double robot_radius = 0.2;
const static double wall_offset = 0.65;
const static int right = 119;
const static int front = 341;
const static int left = 511;
const static int cnt_limit = 50;
const static double bottle_search_range_x_max = 1.5;
const static double bottle_search_range_x_min = 0.0;
const static double bottle_search_range_y_max = 0.0;
const static double bottle_search_range_y_min = -1.5;
const static double ojama_search_range = 0.8;
const static double heikou_thresh = 0.05;
const static double goal_A_y = 0.9;
const static double goal_B_y = 1.5;

//--------------------------------------------
//ctrl-cで停止させるための記述
//--------------------------------------------
bool gIsShuttingDown = false;

void ctrl_c( int aStatus )
{
    gIsShuttingDown = true;
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

void sec_sleep(double t){
    usleep(t*1000000);
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
    Spur_set_angvel( 0.4 );
    Spur_set_angaccel( M_PI / 2.0 );

    Spur_set_pos_GL( 0, 0, 0 );

    return 0;
}

double distance_array(double *x, double *y, int i, int j){
    return sqrt(pow(x[i]-x[j],2.0) + pow(y[i]-y[j],2.0));
}

double distance(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1-x2,2.0) + pow(y1-y2,2.0));
}

int max_of_array(int n[], int len){
  int i, max, ans;

  max = n[0];
  ans = 0;
  for (i=1; i<len; i++) {
    if (max < n[i]){
         max = n[i];
         ans = i;
    }
  }
  return ans;
}

int sei_fu(double a){
    if(a<0)return -1;
    else return 1;
}

//--------------------------------------------
//本体
//--------------------------------------------
int main( int argc , char **argv )
{
    //変数宣言
    int ret;
    S2Port      *urg_port;	//デバイスファイル名
    S2Sdd_t     urg_buff;		//データを確保するバッファ
    S2Scan_t    *urg_data;	//バッファへのポインタ
    S2Param_t   urg_param;	//URGのパラメータを確保

    int phase = start_phase;
    struct velocity
    { 
        double x;
        double w;
    };
    velocity cmd_vel;
    int no_right_wall=0;
    int ojama_counter=0;
    int ojama_r=0;
    int ojama_l=0;
    int bottle_counter=0;
    int goal=0;
    double sum=0;
    double bottle_ave=0;
    int loop_cnt=0;
    int b_cnt[4]={0};

    signal(SIGINT, ctrl_c);	//ctrl-c停止の設定
    
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

    /* init spur and set param */
    if(!debug_laser){
        if(init_spur())return -1;
    }

    //+++++++
    //PCL関係変数の宣言
    //+++++++
    //PointCloudの宣言
    pcl::PointCloud<pcl::PointXYZ> cloud;//URGの点群を保存するクラウドを定義
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);//URGの点群を保存するクラウドを定義
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bottle(new pcl::PointCloud<pcl::PointXYZRGB>);  
    //CloudViewerの宣言
    pcl::visualization::CloudViewer viewer("PCL URG Viewer");  //クラウド表示のためのクラスを定義

    //+++++++
    //メインループ
    //+++++++
    printf("--START--\n");
    while( !gIsShuttingDown && !viewer.wasStopped() ) //ctrl-cかviewerが閉じられるまで繰り返す
    {
        int i=0;
        int k=0;
        int l=0;
        double x_FS[700], y_FS[700], rad_FS[700]; //ロボット座標系の点群
        double x_GL[700], y_GL[700], rad_GL[700]; //絶対座標系の点群
        double dx_FS = 0;
        double dy_FS = 0;
        double dtheta = 0;
        double pole_dist = 0;
        double pole[100][2]={0};
        double tmp[100][2]={0};
    
        ret = S2Sdd_Begin(&urg_buff, &urg_data); //測位データの取り出し

        if( ret > 0 )
        {
            cloud2->clear();	//ポイントクラウドの中身を全部クリアする

            for(unsigned int i=0; i < urg_data->size; ++i) //URGから取得したすべての点について行う
            {
                double rad = ((double)urg_param.step_min + i - urg_param.step_front) * M_PI * 2 / urg_param.step_resolution; //今のステップの角度を計算

                pcl::PointXYZ p;	//１つの点pを定義
                p.x = (double)urg_data->data[i] * cos(rad) / 1000;	//点pのX座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.y = (double)urg_data->data[i] * sin(rad) / 1000;	//点pのY座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                if(urg_data->data[i]==0){ //遠すぎてレーザの点が取れなければ20mにする
                    p.x=20*sei_fu(cos(rad));
                    p.y=20*sei_fu(sin(rad));
                }
                p.z = 0.0; //点pのZ座標を計算(常に0としておく)
                x_FS[i] = p.x;
                y_FS[i] = p.y;
        

                cloud2->points.push_back( p ); //作成した点pをcloudに追加する
            }
            cloud2->width = cloud2->points.size();
            cloud2->height = 1;

            S2Sdd_End(&urg_buff); //バッファのアンロック(読み込み終了)

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud (cloud2);
            vg.setLeafSize (leaf_size, leaf_size, leaf_size);
            vg.filter (*cloud_filtered);
            //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
            
            // Create the filtering object: outlier removal
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            // build the filter
            outrem.setInputCloud(cloud_filtered);
            outrem.setRadiusSearch(removal_radius);
            outrem.setMinNeighborsInRadius (removal_neighbors);
            // apply filter
            outrem.filter (*cloud_filtered);

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (cluster_tolerance);
            ec.setMinClusterSize (min_cluster_size);
            ec.setMaxClusterSize (max_cluster_size);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            // Create and accumulate points
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZRGB>); 

            int j = 0;  
            float colors[12][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}
                                    , {127,0,0}, {0,127,0}, {0,0,127}, {127,127,0}, {0,127,127}, {127,0,127}};  
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
            pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){  
                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                //std::cout << j << " : " << cloud_cluster->width << std::endl;
                for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){  
                    cloud_cluster->points[*pit].r = colors[j%12][0];  
                    cloud_cluster->points[*pit].g = colors[j%12][1];  
                    cloud_cluster->points[*pit].b = colors[j%12][2];  
                    double cluster_centroid_x = cloud_cluster->points[*pit].x;
                    double cluster_centroid_y = cloud_cluster->points[*pit].y;
                    double cluster_centroid_z = cloud_cluster->points[*pit].z;
                    centroid.add (pcl::PointXYZ (cluster_centroid_x, cluster_centroid_y, cluster_centroid_z));
                }
                // Fetch centroid using `get()`
                pcl::PointXYZRGB c1;
                c1.r = 255;
                c1.g = 255;
                c1.b = 255;
                centroid.get (c1);
                cloud_centroid->push_back(c1);
                cloud_cluster->push_back(c1);
                //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
                j++;  
            }
            switch(phase){
                case 0:
                    printf("searching wall\n dtheta = %g\n",dtheta);
                    Spur_spin_GL( M_PI/8 );
                    while( !Spur_near_ang_GL(M_PI/8, M_PI/18.0) )
                    usleep( 5000 );
                    Spur_stop();
                    cmd_vel.x = 0;
                    cmd_vel.w = 0;
  
                    phase = 1;
                    
                    break;
                case 1:
                    dx_FS = x_FS[300]-x_FS[380];
                    dy_FS = y_FS[300]-y_FS[380];
                    dtheta = atan2(dx_FS,dy_FS);
                    printf("APPROACHing wall\n dtheta = %g\n",dtheta);
                    if(fabs(dx_FS)<heikou_thresh){
                        printf("straight\n");
                        cmd_vel.x = 0.2;
                        cmd_vel.w = 0.0;
                        if(x_FS[341]<wall_offset){
                            Spur_stop();
                            cmd_vel.x = 0.0;
                            cmd_vel.w = 0.0;
                            phase++;
                            Spur_set_pos_GL(0,0,0);
                        }
                    }
                    else{
                        printf("searching wall\n");
                        if(x_FS[341]<2.0){
                            cmd_vel.x=0.0;
                            cmd_vel.w=0.3*dx_FS/fabs(dx_FS);
                        }
                        else{
                            cmd_vel.x=0.0;
                            cmd_vel.w=-0.1;
                        }
                    }
                    break;
                case 2:
                    printf("TURN right\n");
                    Spur_spin_GL( -M_PI/2 );
                    while( !Spur_near_ang_GL(-M_PI/2, M_PI/18.0) )
                    usleep( 5000 );
                    /*Spur_set_pos_GL(0,0,0);
                    Spur_line_GL(0.5,0,0);
                    while( !Spur_over_line_GL(0.5, 0.0, 0.0) )
                    usleep( 5000 );
                    */
                    phase++;
                    Spur_set_pos_GL(0,0,0);

                    break;
                case 3:
                    //printf("run along WALL\n");
                    if(Spur_over_line_GL(1.0, 0.0, 0.0) ){
                        for(k=right;k<(right+(front-right)/2);k++){
                            if(y_FS[k]<-0.5)no_right_wall++;
                        }
                        //if(x_FS[front]<0.3)no_right_wall=200;
                    }
                    printf("right wall:%d\n",no_right_wall);
                    //if( !Spur_over_line_GL(2.7, 0.0, 0.0) ){
                    if( no_right_wall<50 ){
                        no_right_wall=0;
                        if(fabs(y_FS[500]-y_FS[650])<heikou_thresh){
                          cmd_vel.x = 0.2;
                          cmd_vel.w = 0.0;
                        }
                        else{
                          cmd_vel.x = 0.0;
                          cmd_vel.w = 0.2*(y_FS[500]-y_FS[650])/fabs(y_FS[500]-y_FS[650]);
                        }
                    }
                    else{
                        no_right_wall=0;
                        Spur_stop();
                        cmd_vel.x = 0.0;
                        cmd_vel.w = 0.0;
                        phase++;
                        Spur_set_pos_GL(0,0,0);
                    }

                    break;
                case 4:
                    printf("waiting for OJAMA_robot:%d\n",ojama_counter);
                    printf("front:%g\n",x_FS[front]);
                    printf("right:%g\n",x_FS[(right+front)/2]);
                    if(x_FS[(right+front)/2]<ojama_search_range && x_FS[front]>ojama_search_range && ojama_counter==0){
                        ojama_counter = 1;
                    }
                    else if(x_FS[(right+front)/2]>ojama_search_range && x_FS[front]<ojama_search_range && ojama_counter==1){
                        ojama_counter = 2;
                    }
                    else if(x_FS[(right+front)/2]<ojama_search_range && x_FS[front]>ojama_search_range && ojama_counter==2){
                        ojama_counter = 3;
                    }

                    if(ojama_counter>=3){
                        for(k=(right+(front-right)/2);k<(front+(left-front)/2);k++){
                            if(x_FS[k]>ojama_search_range)no_right_wall++;
                        }
                        if(no_right_wall>100){
                            printf("GO\n");
                            ojama_counter = 0;
                            phase++;
                        Spur_set_pos_GL(0,0,0);
                        }
                    }

                    no_right_wall=0;
                    cmd_vel.x = 0.0;
                    cmd_vel.w = 0.0;
                    break;
                case 5:
                    printf("searching BOTTLES\n");
                    
                    Spur_line_GL( 2.0, 0.0, 0.0 );
                    while( !Spur_over_line_GL(2.0 - 0.005, 0.0, 0.0) )
                    usleep( 5000 );
                    Spur_stop();
                    
                    Spur_spin_GL( -M_PI/2 );
                    while( !Spur_near_ang_GL(-M_PI/2, M_PI/18.0) )
                    usleep( 5000 );
                    /*Spur_circle_GL( 1.2, -0.80, -0.5 );
                    while( !Spur_over_line_GL( 1.2, -0.80, -M_PI/2 ) )
                    usleep( 5000 );
                    while( !Spur_over_line_GL( 1.2, -0.80, M_PI ) )
                    usleep( 5000 );
                    */
                    Spur_stop();
                    phase++;

                    break;
                case 6:
                    printf("counting BOTTLES\n");
                    //printf("%d\n",cloud_centroid->points.size());
                    cloud_bottle->clear();
                    if(loop_cnt<cnt_limit){
                        for(k=0;k<cloud_centroid->points.size();k++){
                            if(cloud_centroid->points[k].x<bottle_search_range_x_max 
                                && cloud_centroid->points[k].x>bottle_search_range_x_min 
                                && cloud_centroid->points[k].y<bottle_search_range_y_max
                                && cloud_centroid->points[k].y>bottle_search_range_y_min){ 
                                
                                bottle_counter++;
                                cloud_bottle->points.push_back(cloud_centroid->points[k]);
                                sum+=bottle_counter;
                            
                            }
                        }
                        if(bottle_counter==0)b_cnt[0]++;
                        else if(bottle_counter==1)b_cnt[1]++;
                        else if(bottle_counter==2)b_cnt[2]++;
                        else if(bottle_counter==3)b_cnt[3]++;
                        printf("%d bottles\n", bottle_counter);
                        bottle_counter=0;
                        loop_cnt++;
                    }
                    else{
                        /*bottle_ave=sum/loop_cnt;
                        printf("%g\n", bottle_ave);
                        if(max_of_array(b_cnt,4)%2==0){
                            goal=0;
                        }
                        else{
                            goal=1;
                        }
                        sum=0;
                        bottle_counter=0;*/
                        loop_cnt=0;
                        phase++;
                        break;
                    }

                    break;
                case 7:
                    printf("searching BOTTLES\n");
                    
                    Spur_line_GL( 2.0, -1.2, -M_PI/2 );
                    while( !Spur_over_line_GL(2.0, -1.2 + 0.005, -M_PI/2) )
                    usleep( 5000 );
                    Spur_stop();
                    
                    Spur_spin_GL( -M_PI );
                    while( !Spur_near_ang_GL(-M_PI, M_PI/18.0) )
                    usleep( 5000 );
                    Spur_stop();
                    /*Spur_circle_GL( 1.2, -0.80, -0.5 );
                    while( !Spur_over_line_GL( 1.2, -0.80, -M_PI/2 ) )
                    usleep( 5000 );
                    while( !Spur_over_line_GL( 1.2, -0.80, M_PI ) )
                    usleep( 5000 );
                    */
                    phase++;

                    break;
                case 8:
                    printf("counting BOTTLES\n");
                    //printf("%d\n",cloud_centroid->points.size());
                    cloud_bottle->clear();
                    if(loop_cnt<cnt_limit){
                        for(k=0;k<cloud_centroid->points.size();k++){
                            if(cloud_centroid->points[k].x<bottle_search_range_x_max 
                                && cloud_centroid->points[k].x>bottle_search_range_x_min 
                                && cloud_centroid->points[k].y<bottle_search_range_y_max
                                && cloud_centroid->points[k].y>bottle_search_range_y_min){ 
                                
                                bottle_counter++;
                                cloud_bottle->points.push_back(cloud_centroid->points[k]);
                                sum+=bottle_counter;
                            }
                        }
                        if(bottle_counter==0)b_cnt[0]++;
                        else if(bottle_counter==1)b_cnt[1]++;
                        else if(bottle_counter==2)b_cnt[2]++;
                        else if(bottle_counter==3)b_cnt[3]++;
                        printf("%d bottles\n", bottle_counter);
                        bottle_counter=0;
                        loop_cnt++;
                            
                    }
                    else{
                        //bottle_ave=sum/loop_cnt;
                        //printf("%g\n", bottle_ave);
                        if(max_of_array(b_cnt,4)%2==0){
                            goal=0;
                        }
                        else{
                            goal=1;
                        }
                        loop_cnt=0;
                        sum=0;
                        cloud_bottle->clear();
                        bottle_counter=max_of_array(b_cnt,4);
                    
                        Spur_line_GL( 1.2, -1.2, -M_PI );
                        while( !Spur_over_line_GL(1.2 + 0.005, -1.2, -M_PI) )
                        usleep( 5000 );
                        Spur_stop();
                    
                        cmd_vel.x = 0;
                        cmd_vel.w = 0;
                    
                        phase++;
                    
                        break;
                    }

                    break;
                case 9:
                    if(goal==0)
                    printf("There are %d bottles, so GOAL is [A]\n",bottle_counter);
                    else
                    printf("There are %d bottles, so GOAL is [B]\n",bottle_counter);
                    
                    Spur_set_pos_GL(0,0,0);
                    Spur_stop();
                    cmd_vel.x = 0;
                    cmd_vel.w = 0;
                    phase++;

                    break;
                case 10:
                    bottle_counter=0;
                    printf("gate\n");
                    cloud_bottle->clear();
                    for(k=0;k<cloud_centroid->points.size();k++){
                        if(cloud_centroid->points[k].x<1.5
                            && cloud_centroid->points[k].x>0.5 
                            && cloud_centroid->points[k].y<1.0
                            && cloud_centroid->points[k].y>-1.0){ 
                            
                            pole[bottle_counter][0]=cloud_centroid->points[k].x;
                            pole[bottle_counter][1]=cloud_centroid->points[k].y;

                            printf("%g  %g\n",pole[bottle_counter][0],pole[bottle_counter][1]);
                            bottle_counter++;
                            cloud_bottle->points.push_back(cloud_centroid->points[k]);
                        }
                    }
                    
                    printf("before:%g,%g\n",pole[0][0],pole[0][1]);
                    for (k=0;k<2;k++) {
                        for (l=k+1;l<bottle_counter-1;l++) {
                            if (pole[k][1] < pole[l][1]) {
                                tmp[0][1] =  pole[k][1];
                                tmp[0][0] =  pole[k][0];
                                pole[k][1] = pole[l][1];
                                pole[k][0] = pole[l][0];
                                pole[l][1] = tmp[0][1];
                                pole[l][0] = tmp[0][0];
                            }
                        }
                    }
                    
                    printf("after:%g,%g\n",pole[0][0],pole[0][1]);

                    pole_dist = distance(pole[0][0],pole[0][1],pole[1][0],pole[1][1]);
                    printf("pole_dist:%g\n",pole_dist);

                    if(pole_dist>0.7 && pole_dist<0.9){
                        //&& fabs(pole[0][0]-pole[1][0])<0.1){
                        printf("pole\n");
                        Spur_line_GL(pole[0][0]+0.4,(pole[0][1]+pole[1][1])/2,0);
                        while( !Spur_over_line_GL(pole[0][0]+0.4-0.01,(pole[0][1]+pole[1][1])/2,0) )
                        usleep( 5000 );
                        Spur_stop();
                        Spur_set_pos_GL(0,0,0);
                        sec_sleep(1.0);
                        
                        phase++;
                    }
                    else{
                        cmd_vel.x=0;
                        cmd_vel.w=0;
                    }

                    break;
                case 11:
                    if(goal==0){
                        if(y_FS[right]<goal_A_y){
                            Spur_spin_GL(-M_PI/2);
                            while( !Spur_near_ang_GL(-M_PI/2, M_PI/18.0) )
                            usleep( 5000 );
                            Spur_stop();
                            sec_sleep(1.0);
                            Spur_line_GL(0,goal_A_y+y_FS[right],-M_PI/2);
                            while( !Spur_over_line_GL(0,goal_A_y+y_FS[right],-M_PI/2) )
                            usleep( 5000 );
                            Spur_stop();
                            sec_sleep(1.0);
                        }
                        else{
                            Spur_spin_GL(M_PI/2);
                            while( !Spur_near_ang_GL(M_PI/2, M_PI/18.0) )
                            usleep( 5000 );
                            Spur_stop();
                            sec_sleep(1.0);
                            Spur_line_GL(0,goal_A_y+y_FS[right],-M_PI/2);
                            while( !Spur_over_line_GL(0,goal_A_y+y_FS[right],-M_PI/2) )
                            usleep( 5000 );
                            Spur_stop();
                            sec_sleep(1.0);
                        }
                        Spur_spin_GL(0.0 );
                        while( !Spur_near_ang_GL(0.0, M_PI/18.0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                    }
                    else{
                        Spur_spin_GL(M_PI/2);
                        while( !Spur_near_ang_GL(M_PI/2, M_PI/18.0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        Spur_line_GL(0,goal_B_y+y_FS[right],M_PI/2);
                        while( !Spur_over_line_GL(0,goal_B_y+y_FS[right],M_PI/2) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        Spur_spin_GL(0.0 );
                        while( !Spur_near_ang_GL(0.0, M_PI/18.0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                    }

                    bottle_counter=0;
                    printf("garage\n");
                    cloud_bottle->clear();
                    for(k=0;k<cloud_centroid->points.size();k++){
                        if(cloud_centroid->points[k].x<1.5
                            && cloud_centroid->points[k].x>0.5 
                            && cloud_centroid->points[k].y<1.0
                            && cloud_centroid->points[k].y>-1.0){ 
                            
                            pole[bottle_counter][0]=cloud_centroid->points[k].x;
                            pole[bottle_counter][1]=cloud_centroid->points[k].y;

                            printf("%g  %g\n",pole[bottle_counter][0],pole[bottle_counter][1]);
                            bottle_counter++;
                            cloud_bottle->points.push_back(cloud_centroid->points[k]);
                        }
                    }
                    
                    for (k=0;k<2;k++) {
                        for (l=k+1;l<bottle_counter-1;l++) {
                            if (pole[k][1] < pole[l][1]) {
                                tmp[0][1] =  pole[k][1];
                                tmp[0][0] =  pole[k][0];
                                pole[k][1] = pole[l][1];
                                pole[k][0] = pole[l][0];
                                pole[l][1] = tmp[0][1];
                                pole[l][0] = tmp[0][0];
                            }
                        }
                    }
                    
                    if(goal==0){
                        printf("goal A\n");
                        Spur_line_GL(pole[1][0]-0.6,pole[1][1],0);
                        while( !Spur_over_line_GL(pole[1][0]-0.6,pole[1][1],0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        Spur_spin_GL(0.0 );
                        while( !Spur_near_ang_GL(0.0, M_PI/18.0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        
                        phase++;
                    }
                    else{
                        printf("goal B\n");
                        Spur_line_GL(pole[0][0]-0.6,pole[0][1],0);
                        while( !Spur_over_line_GL(pole[0][0]-0.6,pole[0][1],0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        Spur_spin_GL(0.0 );
                        while( !Spur_near_ang_GL(0.0, M_PI/18.0) )
                        usleep( 5000 );
                        Spur_stop();
                        sec_sleep(1.0);
                        
                        phase++;
                    }

                    break;
                case 12:
                    printf("enter\n");
                    /*if(x_FS[front]>0.3 && (fabs(y_FS[front-50]) <0.5 || fabs(y_FS[front+50])<0.5)){
                        cmd_vel.x = 0.05;
                        cmd_vel.w = -2.0*(fabs(y_FS[front-50])-fabs(y_FS[front+50]))*sei_fu(fabs(y_FS[front-50])-fabs(y_FS[front+50]));
                    }
                    else{
                        cmd_vel.x = 0.05;
                        cmd_vel.w = 0.0;
                    }*/

                    printf("%g\n",x_FS[front]);
                    for(k=0;k<cloud_centroid->points.size();k++){
                        if(cloud_centroid->points[k].x<1.0 
                            && cloud_centroid->points[k].x>0.0
                            && cloud_centroid->points[k].y<0.5
                            && cloud_centroid->points[k].y>-0.5){ 
                            
                            cmd_vel.x=0.05;
                            cmd_vel.w=0.2*cloud_centroid->points[k].y;    
                        }
                    }
                    if(x_FS[front]<0.2){
                        Spur_stop();
                        sec_sleep(5);
                        phase++;
                    }

                    break;
                default:
                    printf("others:%d\n",goal);
                    if(!debug_laser)Spur_stop();
                    cmd_vel.x = 0;
                    cmd_vel.w = 0;

                    break;
            }
            //std::cout << j << " : " << cloud_cluster->width << std::endl;
            //viewer.showCloud (cloud2); 
            viewer.showCloud (cloud_cluster); 
            //viewer.showCloud (cloud_centroid);   
            //viewer.showCloud (cloud_bottle); 

            printf("x:%g, w:%g\n",cmd_vel.x, cmd_vel.w);
            if(!debug_laser)Spur_vel(cmd_vel.x, cmd_vel.w);
        }

        else if( ret == -1 ) //戻り値が-1：エラー
        {
            printf("[ERROR] Fatal Error Occured.\n");
        }

        else //戻り値が0：書き込みデータがロック中
        {
            usleep(100);
        }

    }
    printf("--STOP--\n");

    //+++++++
    //終了処理
    //+++++++
    if( Scip2CMD_StopMS( urg_port, &urg_buff ) == 0 ) //URGの測位停止
    {
            printf( "[ERROR] StopMS failed.\n" );
            return 0;
    }
    printf("URG Stopped.\n");

    S2Sdd_Dest( &urg_buff ); //バッファの開放
    printf("Buffer destructed.\n");

    Scip2_Close( urg_port ); //ポートのクローズ
    printf("Port closed.\n");

    return 0;
}
