#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

bool obstacle = false;
double rangeEps = 0;
double angleEps = 0;
ros::Publisher pub;

/**
 * Функция, которая будет вызвана
 * при получении данных от лазерного дальномера
 * параметр функции msg - ссылка на полученное сообщение
 */
void laserCallback(const sensor_msgs::LaserScan& msg)
{
  ROS_DEBUG_STREAM("Laser msg: "<<msg.scan_time);

  //const double kMinRange = 0.3;
  const double wallRange = 1;
  //проверим нет ли вблизи робота препятствия
  //for (size_t i = 0; i<msg.ranges.size(); i++)
  //{
      //if (msg.ranges[i] < kMinRange)
	  //{
		  //obstacle = true;
		  //ROS_WARN_STREAM("OBSTACLE!!!");
		  //break;
	  //}
  //}
  rangeEps = msg.ranges[0] - wallRange;
  angleEps = cos(msg.angle_increment)*msg.ranges[1] - msg.ranges[0];
  ROS_INFO_STREAM("RangeEps: "<<rangeEps<<"; AngleEps: "<<angleEps);
}


/**
 * Функция, которая будет вызвана при
 * получении сообщения с текущем положением робота
 * параметр функции msg - ссылка на полученное сообщение
 */

void poseCallback(const nav_msgs::Odometry& msg)
{
  ROS_DEBUG_STREAM("Pose msg: x = " << msg.pose.pose.position.x<<
          " y = " << msg.pose.pose.position.y <<
          " theta = " << 2*atan2(msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w) );
}
/*
 * функция обработчик таймера
 * параметр функции - структура, описывающая событие таймера, здесь не используется
 */
void timerCallback(const ros::TimerEvent&)
{  
	static int counter = 0;
	counter++;
	ROS_DEBUG_STREAM("on timer "<<counter);
	//сообщение с помощью которого задается
	//управление угловой и линейной скоростью
	geometry_msgs::Twist cmd;
	//при создании структура сообщения заполнена нулевыми значениями
	//если вблизи нет препятствия то задаем команды
    //if (!obstacle)
	//{
        //if (counter % 30 > 15)
		//{
			//ROS_INFO_STREAM("go left");
			//cmd.linear.x = 0.1;
			//cmd.angular.z = 0.5;
		//}
		//else
		//{
			//ROS_INFO_STREAM("go right");
			//cmd.linear.x = 0.1;
			//cmd.angular.z = -0.5;
		//}
	//}
	//отправляем (публикуем) команду
	const double K_A = -20;
	const double K_R = -1;
	cmd.angular.z = K_A * angleEps + K_R*rangeEps;
	cmd.linear.x = 0.1;
	pub.publish(cmd);
}

int main(int argc, char **argv)
{
  /**
   * Инициализация системы сообщений ros
   * Регистрация node с определенным именем (третий аргумент функции)
   * Эта функция должна быть вызвана в первую очередь
   */
  ros::init(argc, argv, "control_node");

  /**
   * NodeHandle  - объект через который осуществляется взаимодействие с ROS:
   * передача сообщений
   * регистрация коллбаков (функций обработки сообщений)
   */
  ros::NodeHandle n;

  /**
   * subscribe() функция подписки на сообщения определенного типа,
   * передаваемое по заданному топику
   * В качестве параметров указываются
   * - топик - на сообщения которого происходит подписка
   * - длина очереди сообщений хранящихся до обработки (если очередь заполняется,
   *  то самые старые сообщения будут автоматически удаляться )
   *  - функция обработки сообщений
   *
   *
   *  Подписываемся на данные дальномера

   */
  ros::Subscriber laser_sub = n.subscribe("base_scan", 1, laserCallback);

  /*
   * Подписываемся на данные о положении робота
   */
  ros::Subscriber pose_sub = n.subscribe("base_pose_ground_truth", 1, poseCallback);


  /*
   * Регистрируем функцию обработчик таймера 10Hz
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), timerCallback);

  /*
   * Сообщаем, что мы будем публиковать сообщения типа Twist по топику cmd_vel
   * второй параметр - длина очереди
   */
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   /**
   * ros::spin() функция внутри которой происходит вся работа по приему сообщений
   * и вызову соответствующих обработчиков . Вся обработка происходит из основного потока
   * (того, который вызвал ros::spin())
   * Функция будет завершена, когда подьзователь прервет выполнение процесса с Ctrl-C
   *
   */
  ros::spin();

  return 0;
}