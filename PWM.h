/****************************************************************************
* Fichier: PWM.h
* Auteurs: Louis-Philippe Chouinard, Olivier St-Onge, Gabriel Helie, Alexandre Vu, Ibrahima Séga Sangaré
* Date: 12 avril 2016
* Description: Classe PWM gerant le comportement du moteur
****************************************************************************/

#ifndef PWM_H
#define PWM_H
#include <avr/interrupt.h>
#include <avr/io.h> 


class Pwm
{
public:

	Pwm();
	~Pwm();
	void Avance(unsigned int pwm);
	void reculer(unsigned int pwm);
 
 //fonction pour les virages à gauche lorsque le robot suit la ligne
	void Tourner_Gauche(unsigned int pwm);
	
  //fonction spéciale pour ajuster les virages à gauche dans les intersections
  void Tourner_GaucheInt(unsigned int pwm);
  
  //fonction pour les virages à droite lorsque le robot suit la ligne
	void Tourner_Droite(unsigned int pwm);
 
 //fonction spéciale pour ajuster les virages à droite dans les intersections
	void Tourner_DroiteInt(unsigned int pwm);
	
  //pivot des roues lors de virages et ajustements
  void pivotDroit(unsigned int pwm);
	void pivotGauche(unsigned int pwm);

};

#endif

