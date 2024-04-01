<p>STERO
Michał Kwarciski, Kacper Marchlewicz
<br>Część manipulacyjna
<p>Projekt 1 <br>Dokumentacja  <br>
  <br>
Celem projektu jest implementacja, w Pythonie przy użyciu systemu Ros, algorytmu pozwalającego robotowi Velma zasymulować wykonania zadania PickAndPlace. W świecie symulacji stworzyliśmy 2 stoły i kostkę. 
</p
<p>Implementacja: <br> 
Lista metod klasy zawierającej funkcje Velmy:  <br>
__ini__() - inicjalizacja interfejsu Velmy, planera i octomapy, bazowanie robota  <br>
homing() - funkcja służąca do bazowania robota  <br>
goto_start_position() - funkcja wywoływana przez homing(), zadaje odpowiednie pozycje stawów  <br>
normalize_angle() - funkcja wywoływana przez move_head(), normalizuje kąt pomiędzy obecnym i zadanym  <br>
move_head() - funkcja wywoływana przez move_around(), służy do poruszania głową Velmy  <br>
move_around() - funkcja służąca do wykonania ruchów głową Velmy, aby odkryć otoczenie przez robotem  <br>
joint_mode() - funkcja służąca do przełączenia Velmy w tryb jnt_imp  <br>
cart_mode() - funkcja służąca do przełączenia Velmy w tryb cart_imp  <br>
change_gripper_state() - funkcja służąca do otwierania, bądź zamykania chwytaków  <br>
cart_move() - funkcja służąca do poruszania w układzie kartezjańskim (CIMP)  <br>
plan_and_execute() - funkcja służąca do wyznaczenia i wykonania planu dla ruchu w przestrzeni stawów (JIMP)   <br> 
inv_kin() - funkcja służąca do obliczenia kinematyki odwrotnej w celu znalezienia trajektorii do celu  <br>
transformations_object() - funkcja służąca do obliczenia transformaty pozycji obiektu  <br>
find_the_best_corner() - funkcja służąca do znalezienia punktu na docelowym stole, gdy jego środek jest poza zasięgiem  <br>
</p
<p>Plik launch  
Znajduje się w nim:<br>  
- inicjalizacja utworzonego przez nas świata<br>
- uruchomienie systemu Velmy<br>
- publikowanie pozycji klocka i obu stołów<br>
- uruchomienie odpowiedniej konfiguracji rviz<br>
- uruchomienie offline severa octomapy z utworzoną wcześniej octomapą<br>
</p
<p>Wykonanie programu:<br>
Program zaczyna się od inicjalizacji robota. <br>  
Po poprawnej inicjalizacji następuje wejście robota w tryb ruchu w przestrzeni stawów.  <br> 
Popranie pozycji pudełka i stołów.  <br>
Zaplanowanie i wykonanie trajektorii nad pudełko.  <br>
Zmianę trybu ruchu na tryb kartezjański.  <br>
Delikatne obniżenie się do obiektu.  <br>
Chwycenie obiektu.  <br>
Delikatne podniesienie obiektu.  <br>
Zaplanowanie i wykonanie trajektorii do celu.  <br>
Puszczenie obiektu.  <br>
Podniesienie ramienia.  <br>
Powrót do pozycji początkowej.    <br>
Gotowość do ponownego uruchomienia.  <br>
</p
<p>Uruchomienie programu:  
Uruchomienie programu wymaga wykonania poniższych komend w różnych terminalach:  <br>
roscore - uruchomienie roscore  <br>
roslaunch rcprg_gazebo_utils gazebo_client.launch - uruchomienie gazebo  <br>
roslaunch stero_manipulation proj_1.launch - uruchomienie pliku launch <br> 
rosrun stero_manipulation proj_1.py - uruchomienie algorytmu  <br>
</p
<p>Prezentacja działania
octomapa rviz<br>

![images](images/01.png "01")

początkowe ustawienie świata<br>
![images](images/1.png "1")
ruch w kierunku kostki<br>
![images](images/2.png "2")
opuszczenie do kostki<br>
![images](images/3.png "3")
pochwycenie kostki<br>
![images](images/4.png "4")
uniesienie obiektu<br>
![images](images/5.png "5")
przeniesienie do celu<br>
![images](images/6.png "6")
odłożenie obiektu<br>
![images](images/7.png "7")
powrót do pozycji początkowej<br>
![images](images/8.png "8")
analogiczna kolejność dla drugiej ręki<br>
![images](images/a.png "a")
![images](images/b.png "b")
![images](images/c.png "c")
![images](images/d.png "d")
![images](images/e.png "e")
</p
</p

</p
<p>Diagramy SysML
Projekt 1<br>
<img src="images/sysml/Projekt1 wymagania Requirement Diagram.jpg" />
<img src="images/sysml/Świat Block Definition Diagram2.jpg" />
<img src="images/sysml/Projekt1 Diagram.jpg" />
</p

<p>Projekt 2 <br>Dokumentacja  <br>
  <br>
Celem projektu jest implementacja, w Pythonie przy użyciu systemu Ros, algorytmu pozwalającego robotowi Velma zasymulować otwieranie drzwi szafki.
</p
<p>Implementacja: <br> 
Funkcje użyte jak w projekcie 1.
Dodatkowe funkcje: <br>
get_PYKDL_frame() - funkcja zwracająca obiekt PyKDL o przesunięciu o zadane wartość w osiach x, y, z, bez rotacji<br>
moveCardImp() - funkcja służąca do wykonania prostego ruchu w płaszyśnie kartezjańskiej z możliwością modykikacji impedancji stawu. <br>
</p
<p>Plik launch - taki sam jak w projektcie 1.
</p
<p>Wykonanie programu:<br>
Program zaczyna się od inicjalizacji robota. <br>  
Po poprawnej inicjalizacji następuje wejście robota w tryb ruchu w przestrzeni stawów.  <br> 
Popranie pozycji klamek i środków masy drzwi szafki.  <br>
Wybór klamki i ręki, którą bedzie wykonywanie zadanie. <br>
Zaplanowanie i wykonanie trajektorii obok klamki.  <br>
następuje wejście robota w tryb ruchu w przestrzeni kartezjańskiej.  <br> 
Przymkięcie palców i podsunięcie się do klamki.  <br>
Złapanie klamki. <br>
Niepełne otwarcie szafki. <br>
Puszczenie szafki. <br>
Przesunięcie się ręki robota do pozycji, w której może "popchnąć" szfkę, żeby otrzorzyła się na zadaną szerokość. <br>
Popchnięcie szafki. <br>
Zmiana trybu pracy robota na ruch w przestrzeni stawów. <br>
Powrót do pozycji początkowej.    <br>
Gotowość do ponownego uruchomienia.  <br>
</p
<p>Uruchomienie programu:  
Uruchomienie programu wymaga wykonania poniższych komend w różnych terminalach:  <br>
roscore - uruchomienie roscore  <br>
roslaunch rcprg_gazebo_utils gazebo_client.launch - uruchomienie gazebo  <br>
roslaunch stero_manipulation proj_1.launch - uruchomienie pliku launch <br> 
rosrun stero_manipulation proj_2.py - uruchomienie algorytmu  <br>
</p
<p>Prezentacja działania
   <img src="images/0.png" />
 <img src="images/20.png" />
   <img src="images/21.png" />
  <img src="images/22.png" />
   <img src="images/23.png" />
   <img src="images/24.png" />
   <img src="images/25.png" />
   <img src="images/26.png" />
   <img src="images/27.png" />
   <img src="images/28.png" />
   <img src="images/29.png" />
   <img src="images/30.png" />
   <img src="images/31.png" />
   <img src="images/32.png" />
</p
<p>Diagramy SysML
Projekt 2<br>
<img src="images/sysml/Projekt2 wymagania Requirement Diagram.jpg" />
<img src="images/sysml/Świat Block Definition Diagram2.jpg" />
<img src="images/sysml/szafka Block Definition Diagram.jpg" /> 
<img src="images/sysml/Projekt2 Diagram.jpg" />

</p
<p>Diagramy SysML
Wspólne dla obu projektów<br>
<img src="images/sysml/Velma Block Definition Diagram.jpg" />
<img src="images/sysml/Projekt Block Definition Diagram.jpg" />
<img src="images/sysml/INIT.jpg" />
<img src="images/sysml/plan_and_execute.jpg" />
<img src="images/sysml/move_cart_imp.jpg" />
<img src="images/sysml/GetPYKDLFrame.jpg" />
<img src="images/sysml/joint_mode.jpg" />
<img src="images/sysml/inv_kin.jpg" />
<img src="images/sysml/homing.jpg" />
<img src="images/sysml/change_gripper_state.jpg" />
<img src="images/sysml/cart_move.jpg" />
<img src="images/sysml/cart_mode.jpg" />

</p
