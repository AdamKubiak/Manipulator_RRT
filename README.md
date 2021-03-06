# Manipulator RRT
Projekt wykonany na zajęcia projektowe Sterowników Robotów na Politechnice Wrocławskiej

# Opis projektu
Założeniem projektu jest budowa oraz oprogramowanie manipulatora RRT i zaprojektowanie interfejsu
dla operatora.
## Opis funkcjonalności robota i pilota:
- [X] Konstrukcja samego manipulatora jest wykonana z gotowych aluminiowych elementów wykonanych pod serwomechanizmy, takich jak uchwyty, mocowania, orczyki i wydrukowanego elementu, który jest odpowiedzialny za ruch translacyjny manipulatora.
- [X] Jako punkt pracy manipulatora wykorzystano mały elektromagnez, odpowiedzialny za przenoszenie małych metalowych elementów.
- [X] Do sterowania manipulatora została wykorzystana odwrotna kinematyka oraz joysticki, które będą umieszone na pilocie operatorskim.
- [X] Pilot operatorski, dzięki wyświetlaczowi OLED, daje możliwość wglądu do najważniejszych informacji tj. informacja zwrotna na temat położenia punktu pracy manipulatora czy kąt rotacji każdego z przegubów. Jednak jego główną rolą jest sterowanie położeniem samego ramienia.
- [X] Współrzędne ustawione przez użytkownika można można zapisywać do sekwencji, a następnie ją odtwarzać. Wszystkie sekwencje będą zapisywane do pamięci FLASH.
- [X] Dane do współrzędnych mogą być wprowadzane za pomocą klawiatury matrycowej lub przy pomocy joysticków.
- [X] Do kominikacji między płytką odpowiedzialną za obsługę manipulatora i tą na pilocie wykorzystano moduł Bluetooth 4.0

![](docs/img/manip_rel.jpg)
![](docs/img/pilot_rel.jpg)

# Elektronika
## Schematy wykonanej elektroniki
## Pilot
![](docs/img/pilot.png)
## Manipulator
![](docs/img/robot.png)
