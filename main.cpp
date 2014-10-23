#include "include/defs/defs.h"
#include "include/board/boardDynamixel.h"
#include "include/kinematic/kinematicLie.h"
#include "include/legModel/insectLeg.h"
#include "include/robotModel/robotMessor2.h"
#include "include/visualization/visualizerGL.h"
#include <iostream>
#include <stdio.h>


/*
 Paulina Jankowska


 */

using namespace std;

/** Klasa pokazowa dla dokumentacji doxygen
 *
 * Klasa pokazowa stworzona w celu zademonstrowania dzia³ania dokumentacji w kodzie. Nie jest ona przeznaczona do u¿ycia.
 *
 * @authors Paulina Jankowska, Tomasz Chroœniak
 */
class DoxygenTestClass
{

private:
  /** Prywatny atrybut.
   *
   * Elementy prywatne nie s¹ wyœwietlane w generowanej dokumentacji.
   */
  int member1;

  /** Prywatna metoda.
   *
   * Zgodnie ze stwierdzeniem przy prywatnym atrybucie, elementy prywatne nie s¹ zawarte w dokumentacji.
   *
   * @param param1 parametr
   * @see member2
   */
  void privateMethod1(int param1);

protected:
  char member2; /**< Atrybut chroniony z dokumentacj¹ w tej samej linii (inline). W przypadku tego rodzaju dokumentacji wszystko co znajduje siê przed pierwsz¹ kropk¹ jest traktowane jako opis skrócony. Ca³a reszta jest traktowana jako opis szczegó³owy. Mo¿na tu u¿ywaæ linków, np: {@link publicMember1}.  */

public:
  /** Atrybut publiczny z dokumentacj¹ w stylu pe³nym.
   *
   * Podobnie jak w przypadku dokumentacji inline, wszystko po pierwszej kropce jest traktowane jako opis szczegó³owy.
   * Do opisu szczegó³owego mo¿na np. wklejaæ fragmenty kodu, jak poni¿ej:
   *
   *     metoda2(param1,param2);
   *
   * Taki fragment musi byæ poprzedzony czterema spacjami. Zostanie on oznaczony na stronie w odpowiedni sposób.
   * W dowolnym momencie mo¿emy odwo³aæ siê do jakiegoœ
   */
  int publicMember1;

  /** Tutaj znajduje siê przyk³ad publicznej metody.
   *
   * Dostêpne s¹ takie polecenia jak param, return. Pozwalaj¹ one na dodanie opisu do parametrów oraz opisanie wartoœci zwracanych przez dan¹ metodê.
   * Do tego dostêpne jest tak¿e polecenie throw (lub throws), które pozwala okreœliæ rodzaj wyj¹tku, który mo¿e wyst¹piæ.
   *
   * @param parametr1 Parametry mog¹ zostaæ opisane przy u¿yciu komendy param. Nieopisane parametry nie wyœwietl¹ siê na liœcie parametrów. Dodatkowo doxygen ostrze¿e nas, ¿e jeden z parametrów nie posiada w³asnej dokumentacji.
   * @throws NullPointerException Opis wyj¹tku sk³ada siê z nazwy wyj¹tku oraz opisu.
   * @throws IOException
   * @return Tutaj mo¿na umieœciæ opis wartoœci zwracanej.
   * @see Tutaj mo¿na umieœciæ linki do innych zdefiniowanych elementów.
   */
    virtual int metoda1(int parametr1, double parametr2);

    /** Tutaj znajduje siê opis kolejnej metody.
     *
     * @see publicMember1
     */
    int metoda2(int parametr1/**< [in] opis parametru wejœciowego mo¿na tak¿e umieœciæ w linii, za parametrem. Dodatkowo mo¿na u¿yæ opcjonalnego okreœlenia kierunku przep³ywu danych dla danego parametru. */, double parametr2/**< [out] opis parametru wyjœciowego umieszczony w linii */)
    {
      publicMember1 = parametr1;
      return metoda1(parametr1,parametr2);
    }

    DoxygenTestClass();
    virtual ~DoxygenTestClass();

};

int main( int argc, const char** argv )
{
    try {
         Board* board;
         board = createBoardDynamixel();
         std::cout << "Board type: " << board->getName() << "\n";

         Kinematic* kinematicModel;
         kinematicModel = createKinematicLie("../resources/legModel.xml");
         std::cout << "Kinematic type: " << kinematicModel->getName() << "\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
