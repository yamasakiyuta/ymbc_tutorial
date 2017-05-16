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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
//RPP
#include"scip2awd.h" //ロボ研のURGドライバ(NewPCでインストール済み)

const static double leaf_size = 0.01; //1cm
const static double cluster_tolerance = 0.1; //10cm
const static int min_cluster_size = 10;
const static int max_cluster_size = 700;
const static double removal_radius = 0.05;
const static int removal_neighbors = 2;
//--------------------------------------------
//ctrl-cで停止させるための記述
//--------------------------------------------
bool gIsShuttingDown = false;

void ctrl_c( int aStatus )
{
    gIsShuttingDown = true;
    signal(SIGINT, NULL);
}

//--------------------------------------------
//本体
//--------------------------------------------
int main( int argc , char **argv )
{
    signal(SIGINT, ctrl_c);	//ctrl-c停止の設定

    //URG関係の変数宣言
    S2Port      *urgPort;	//デバイスファイル名
    S2Sdd_t     urgBuf;		//データを確保するバッファ
    S2Scan_t    *urgData;	//バッファへのポインタ
    S2Param_t   urgParam;	//URGのパラメータを確保

    //+++++++
    //URGポートのオープン
    //+++++++
    if( argc >= 2 ) //実行時に引数が入力されている場合は、最初の引数のデバイス名を指定してURGをオープンする　そうでなければデフォルト
    {
        urgPort = Scip2_Open( argv[1], B115200 ); //デバイス名,ボーレート設定を指定してオープン
    }
    else
    {
        urgPort = Scip2_Open( "/dev/ttyACM0", B115200 ); //デバイス名(デフォルト),ボーレート設定を指定してオープン
    }

    if( urgPort == 0 ) //ポートが開けなかった場合はエラーメッセージを表示して終了
    {
        printf("[ERROR] Failed to Open Device! \n");
        return 0;
    }
    sleep(1);
    printf("Device Opened.\n");

    S2Sdd_Init( &urgBuf );	//バッファの初期化
    printf("Buffer initialized.\n");

    Scip2CMD_PP(urgPort, &urgParam); //URGパラメータの読み出し
    Scip2CMD_StartMS( urgPort, urgParam.step_min, urgParam.step_max, 1, 0, 0, &urgBuf, SCIP2_ENC_3BYTE ); //垂れ流しモードの開始

    //+++++++
    //PCL関係変数の宣言
    //+++++++
    //PointCloudの宣言
    pcl::PointCloud<pcl::PointXYZ> cloud;//URGの点群を保存するクラウドを定義
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);//URGの点群を保存するクラウドを定義
    //CloudViewerの宣言
    pcl::visualization::CloudViewer viewer("PCL URG Viewer");  //クラウド表示のためのクラスを定義

    //+++++++
    //メインループ
    //+++++++
    printf("--START--\n");
    while( !gIsShuttingDown && !viewer.wasStopped() ) //ctrl-cかviewerが閉じられるまで繰り返す
    {

        int ret = S2Sdd_Begin(&urgBuf, &urgData); //測位データの取り出し

        if( ret > 0 )
        {
            cloud2->clear();	//ポイントクラウドの中身を全部クリアする

            for(unsigned int i=0; i < urgData->size; ++i) //URGから取得したすべての点について行う
            {
                double rad = ((double)urgParam.step_min + i - urgParam.step_front) * M_PI * 2 / urgParam.step_resolution; //今のステップの角度を計算

                pcl::PointXYZ p;	//１つの点pを定義
                p.x = (double)urgData->data[i] * cos(rad) / 1000;	//点pのX座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.y = (double)urgData->data[i] * sin(rad) / 1000;	//点pのY座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
                p.z = 0.0; //点pのZ座標を計算(常に0としておく)

                cloud2->points.push_back( p ); //作成した点pをcloudに追加する
            }
            cloud2->width = cloud2->points.size();
            cloud2->height = 1;

            S2Sdd_End(&urgBuf); //バッファのアンロック(読み込み終了)

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
            std::cout << j << " : " << cloud_cluster->width << std::endl;
            viewer.showCloud (cloud_cluster);   
            //viewer.showCloud (cloud_centroid.makeShared());   

            //viewer.showCloud( cloud.makeShared() ); //作成したcloudを表示 makeSharedメソッドを使うとpcl::PointCloud<pcl::PointXYZ>::Ptrを引数として取る関数にアクセスできる

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
    if( Scip2CMD_StopMS( urgPort, &urgBuf ) == 0 ) //URGの測位停止
    {
            printf( "[ERROR] StopMS failed.\n" );
            return 0;
    }
    printf("URG Stopped.\n");

    S2Sdd_Dest( &urgBuf ); //バッファの開放
    printf("Buffer destructed.\n");

    Scip2_Close( urgPort ); //ポートのクローズ
    printf("Port closed.\n");

    return 0;
}
