#include <ros/ros.h>
#include <std_msgs/UInt16.h>

void show_menu() {
    
        std::cout << "\x1B[2J\x1B[H";
        std::cout << "Select algorythm:" << std::endl;
        std::cout << "\t0. Dummy" << std::endl;
        std::cout << "\t1. Voyager" << std::endl;
        std::cout << "\t2. Wall Follower" << std::endl;
        std::cout << "\t3. Wall Follower v2" << std::endl;
        std::cout << "Type -1 to exit" << std::endl;

        std::cout << "Your choose: ";
}

int run_menu(){
    show_menu();
    int choose;
    std::cin >> choose;
    return choose;
}

int main(int argc, char **argv)
{

    /**
     * Инициализация системы сообщений ros
     * Регистрация node с определенным именем (третий аргумент функции)
     * Эта функция должна быть вызвана в первую очередь
     */
    ros::init(argc, argv, "console_client");

    /**
     * NodeHandle  - объект через который осуществляется взаимодействие с ROS:
     * передача сообщений
     * регистрация коллбаков (функций обработки сообщений)
     */
    ros::NodeHandle node("console_client");

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

    ros::Publisher algo_pub = node.advertise<std_msgs::UInt16>("/selector", 100);

    for (int choose = 0; choose != -1; choose = run_menu())
    {
        std_msgs::UInt16 msg;
        msg.data = choose;

        algo_pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}