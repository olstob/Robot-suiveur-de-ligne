/*
Fichier: delControl.cpp
Auteur(s): Louis-Philippe Chouinard, Gabriel Helie, Olivier St-Onge, Ibrahima Sega Sangare, Alexandre Vu
Date de creation: 12 avril 2016
Description: Implementation de la classe delControl

 */
#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>
#include "delControl.h"
#include <avr/interrupt.h>

Del::Del()
{
    PORTA = 0x00;
}
Del::~Del(){}

/****************************************************************************
* Fonction:	allumerRouge
* Description:  Permet d'allumer la Del rouge et d'ajuster l'intensite
* Paramètres:	uint8_t tempsEteint : Temps où la Del est éteinte
* Retour:	Aucune valeur de retour
****************************************************************************/
void Del::allumerRouge(uint8_t tempsEteint)
{
    PORTA = 0x01;
    _delay_ms(1);
    for(uint8_t i = 0; i < tempsEteint; i++)
    {
        PORTA = 0x00;
        _delay_ms(1/2);
    }
}
/****************************************************************************
* Fonction:	allumerVert
* Description:  Permet d'allumer la Del verte et d'ajuster l'intensite
* Paramètres:	uint8_t tempsEteint : Temps où la Del est éteinte
* Retour:	Aucune valeur de retour
****************************************************************************/
void Del::allumerVert(uint8_t tempsEteint)
{
    PORTA = 0x02;
    _delay_ms(1);
    for(uint8_t i = 0; i < tempsEteint; i++)
    {
        PORTA = 0x00;
        _delay_ms(1/2);
    }
}

/****************************************************************************
* Fonction:	allumerAmbree
* Description:  Permet d'allumer la Del ambree et d'ajuster l'intensite
* Paramètres:	uint8_t tempsEteint : Temps où la Del est éteinte
* Retour:	Aucune valeur de retour
****************************************************************************/
void Del::faireAmbree(uint8_t tempsEteint)
{
    for(uint8_t i = 0; i < 255; i++)
        PORTA = 0x02; 
    for(uint8_t i = 0; i < 20; i++)
        PORTA = 0x01;
    for(uint8_t i = 0; i < tempsEteint; i++)
    {
        PORTA = 0x00;
        _delay_ms(1/2);
    }
}

/****************************************************************************
* Fonction:	eteindreDel
* Description:  Permet d'éteindre la Del
* Paramètres:	Aucun
* Retour:	Aucune valeur de retour
****************************************************************************/
void Del::eteindreDel()
{
    PORTA = 0x00;
}

