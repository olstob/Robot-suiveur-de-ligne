/*
Fichier: delControl.h
Auteur(s): Louis-Philippe Chouinard, Gabriel Helie, Olivier St-Onge, Ibrahima Sega Sangare, Alexandre Vu
Date de creation: 12 avril 2016
Description: Description de la classe delControl
*/
#ifndef DEL_H
#define CDEL_H
#include <avr/interrupt.h>
#include <avr/io.h> 



class Del
{
public:

   Del();
   ~Del();
    void allumerRouge(uint8_t tempsEteint);
    void allumerVert(uint8_t tempsEteint);
    void faireAmbree(uint8_t tempsEteint);
    void eteindreDel();
   

private:
};

#endif

