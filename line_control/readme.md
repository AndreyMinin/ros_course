## Практическое задание №3 
### Управление движением робота по траектории  
(команды выполняются при нахождении в директории рабочего пространства)  
0. Скачиваем проект:
```bash
git clone <адрес репозитария> src
```
или обновляем исходные файлы(если проект уже скачан):
```bash
cd src  
git pull  
cd ..  
```
1. Сборка проекта (из рабочей директории):
```bash
catkin_make
```
2. Не забываем в каждом терминале, где планируем запускать бинарные файлы проекта, инициализировать рабочее пространство:  
```bash
source devel/setup.bash
```
3. Запуск модели робота и управляющего кода для движения робота по траектории:  
```bash
roslaunch line_control line.launch
```
Должно открыться окно симулятора Stage с роботом в пустом мире. Параллельно запускается управляющий код с параметрами, заданными в launch файле. Робот после запуска начинает движение вдоль прямой `y = -7`. Движение колебательное, расходящееся.  
4. Изменяем парметр в launch файле `prop_factor`. Нужно подобрать значение, чтобы движение робота вдоль прямой стало ассимптотически устойчивым. (Спойлер: подойдет `prop_factor=2`). Перезапускаем модель и управляющий код
```bash
Ctrl-C
roslaunch line_control line.launch
```
5. Изменяем код в файле line_control/src/line_control.cpp: в строке 69 меняем вызов функции `cross_track_error_line` на `cross_track_error_circle`. Собираем проект и перезапускаем:  
```bash
catkin_make
roslaunch line_control line.launch
```
Наблюдаем, что робот начал двигаться вдоль окружности с центром в точке (-6, 0) и радиусом 7 м. Нужно подобрать коэффициенты, чтобы движение было устойчивым с минимальной ошибкой.  
6. Запускаем управление вдоль окружности и утилиту rqt:  
```bash
rqt
```
Открываем плагин `rqt_plot`: plugins->visualization->plot. В строке topic указываем `/err` - топик, в который наше приложение отправляет текущую ошибку управления. Если удалось хорошо подобрать пропорциональный и дифференциальный коэффициенты, то при движении по окружности мы увидим постоянную (статическую) ошибку. Нужно подобрать интегральную составляющую регулятора для уменьшения оставшейся статической ошибки.  
  
  
## Задача
1. Подобрать коэффициенты регулятора для устойчивого движения робота по траектории
2. Реализовать еще одну функцию вычисления ошибки, чтобы робот двигался по овальной траектории, задаваемой двумя окружностями одинакового радиуса и прямыми, параллельными оси x. Wентры окружностей лежат на оси x, а прямые являются касательными к окружностям. Расстояние между центрами окружностей равно удвоенному радиусу.