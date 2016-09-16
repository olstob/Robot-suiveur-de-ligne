/****************************************************************************
* Fichier: PWM.cpp
* Auteurs: Louis-Philippe Chouinard, Olivier St-Onge, Gabriel Helie, Alexandre Vu, Ibrahima Séga Sangaré
* Date: 12 avril 2016
* Description: Implémentation de la classe PWM
****************************************************************************/

#include <avr/io.h>
#define F_CPU 8000000
#include <util/delay.h>
#include "PWM.h"


Pwm::Pwm()
{
		TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10); 
    
		TCNT1 = 0;

		OCR1A = 0 ;

		OCR1B = 0 ;

		TCCR1B |= (1<<CS11) ;

		TCCR1C = 0;
}

Pwm::~Pwm()
{}


/****************************************************************************
* Fonction:	Avance
* Description: permet de faire avancer le robot
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::Avance(unsigned int pwm)
{
	PORTD = 0x00;
	OCR1A = pwm * 1.05;
	OCR1B = pwm;

}


/****************************************************************************
* Fonction: reculer
* Description: permet de faire reculer le robot à la vitesse voulue
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::reculer(unsigned int pwm)
{
	PORTD = 0xc0;	//Les deux roues en mode reculons
	OCR1A = OCR1B = pwm;

}

/****************************************************************************
* Fonction:	TournerGauche
* Description: prendre un virage à gauche
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::Tourner_Gauche(unsigned int pwm)
{
    PORTD = 0x00;
	OCR1A = pwm * 1.05 / 1.5;
	OCR1B = pwm;
}

/****************************************************************************
* Fonction:	TournerGaucheInt
* Description: prendre un virage à gauche lorsque le robot est dans une intersection
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::Tourner_GaucheInt(unsigned int pwm)
{
	PORTD = 0x00;
	OCR1A = pwm * 1.05 / 2;
	OCR1B = pwm;
}

/****************************************************************************
* Fonction:	TournerGauche
* Description: prendre un virage à droite
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::Tourner_Droite(unsigned int pwm)
{
    PORTD = 0x00;
	OCR1A = pwm * 1.05;
	OCR1B = pwm / 1.5;
}

/****************************************************************************
* Fonction:	TournerDroiteInt
* Description: prendre un virage à droite lorsque le robot est dans une intersection
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::Tourner_DroiteInt(unsigned int pwm)
{
	PORTD = 0x00;
	OCR1A = pwm * 1.05;
	OCR1B = pwm / 2;
}

/****************************************************************************
* Fonction:	pivotDroit
* Description: faire pivoter le robot à droite rapidement plus longtemps pour les coins et les ajustements de trajectoire
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::pivotDroit(unsigned int pwm)
{
	PORTD = 0x40;	//La roue droite en mode reculons
    OCR1A = pwm * 1.05;
    OCR1B = pwm;
}

/****************************************************************************
* Fonction:	pivotGauche
* Description: faire pivoter le robot à gauche rapidement plus longtemps pour les coins et les ajustements de trajectoire
* Paramètres:	- unsigned int pwm: contrôle du duty cycle ou vitesse du robot
* Retour:		aucun
****************************************************************************/
void Pwm::pivotGauche(unsigned int pwm)
{
	PORTD = 0x80;	//La roue gauche en mode reculons
    OCR1A = pwm * 1.05;
    OCR1B = pwm;
}


