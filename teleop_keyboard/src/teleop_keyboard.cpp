#include "teleop_keyboard/teleop_keyboard.hpp"
#include "teleop_keyboard/df_keyboard.hpp"
#include <fcntl.h> // For file control
#include <thread>
#include <cstdlib>
void setNonBlockingInput()
{
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);
  ttystate.c_lflag &= ~(ICANON | ECHO);
  ttystate.c_cc[VMIN] = 0;
  ttystate.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

TeleopKeyboard::TeleopKeyboard()
    : Node("teleop_keyboard"),
      target_linear_velocity_(0.0),
      target_angular_velocity_(0.0),
      key('\0'),
      flag_key(false)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = this->create_wall_timer(
      10ms, std::bind(&TeleopKeyboard::publish_command, this));
  start_display();
  std::shared_ptr<std::thread> read_key_thread = std::make_shared<std::thread>(&TeleopKeyboard::read_key, this);
  (*read_key_thread).detach();
}

void TeleopKeyboard::start_display()
{
  std::cout << "        w" << '\n';
  std::cout << "   a    s    d" << '\n';
  std::cout << "        x" << '\n';
  std::cout << "w : Forward" << '\n';
  std::cout << "x : Reverse" << '\n';
  std::cout << "a : Left turn" << '\n';
  std::cout << "d: Right turn" << '\n';
  std::cout << "s : Stop" << '\n';
  std::cout << "c : Display Clear" << '\n';
}

void TeleopKeyboard::read_key(){
  struct termios old_settings, new_settings;
  tcgetattr(STDIN_FILENO, &old_settings);
  new_settings = old_settings;
  // 터미널 모드 변경
  new_settings.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
  while (true)
  {
    signal(SIGINT, SIG_DFL);
    if (kbhit())
    {
      // 키 입력 처리
      key = getchar();
      // std::cout << "입력받은 키: " << key << std::endl;
      flag_key = true;
    }
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}
int TeleopKeyboard::kbhit(){
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

void TeleopKeyboard::publish_command(){
  auto msg = std::make_shared<geometry_msgs::msg::Twist>();
  if (flag_key){
    switch (key){
    case 'w':
      target_linear_velocity_ += LIN_VEL_STEP_SIZE;
      target_linear_velocity_ = check_linear_limit_vel(target_linear_velocity_);
      system("clear");
      start_display();
      print_vels();
      break;
    case 'x':
      target_linear_velocity_ -= LIN_VEL_STEP_SIZE;
      target_linear_velocity_ = check_linear_limit_vel(target_linear_velocity_);
      system("clear");
      start_display();
      print_vels();
      break;
    case 'a':
      target_angular_velocity_ += ANG_VEL_STEP_SIZE;
      target_angular_velocity_=check_angular_limit_vel(target_angular_velocity_);
      system("clear");
      start_display();
      print_vels();
      break;
    case 'd':
      target_angular_velocity_ -= ANG_VEL_STEP_SIZE;
      target_angular_velocity_=check_angular_limit_vel(target_angular_velocity_);
      system("clear");
      start_display();
      print_vels();
      break;
    case 's':
      target_linear_velocity_ = 0.0;
      target_angular_velocity_ = 0.0;
      system("clear");
      start_display();
      print_vels();
      break;
    default:
      break;
    }
    msg->linear.x = check_linear_limit_vel(target_linear_velocity_);
    msg->linear.y = 0.0;
    msg->linear.z = 0.0;

    msg->angular.x = 0.0;
    msg->angular.y = 0.0;
    msg->angular.z = check_angular_limit_vel(target_angular_velocity_);
    prev_twist_ = *msg;
    flag_key = false;
  }
  else{
    // No user input, send zero velocity
    (*msg) = prev_twist_;
  }
  publisher_->publish((*msg));
}

void TeleopKeyboard::print_vels(){
  std::cout << "currently:\tlinear velocity " << target_linear_velocity_
            << "\t angular velocity " << target_angular_velocity_ << std::endl;
}
double TeleopKeyboard::check_linear_limit_vel(double vel){
  return std::max(std::min(vel, MAX_LIN_VEL), -MAX_LIN_VEL);
}

double TeleopKeyboard::check_angular_limit_vel(double vel){
  return std::max(std::min(vel, MAX_ANG_VEL), -MAX_ANG_VEL);
}

TeleopKeyboard::~TeleopKeyboard(){
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;

    // 터미널 모드 변경
    new_settings.c_lflag |= ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}