#include <Arduino.h>
#include "CmdProcessor.h"

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

ASerialCommandProcessor::ASerialCommandProcessor(
    const ACommandHandler* handlers, size_t numberOfHandlers
)
    : Handlers( handlers )
    , NumberOfHandlers( numberOfHandlers )
    , ActiveHandler( numberOfHandlers )
    , BodyLength( 0 )
    , NotRecognized( false )
    , Rejected( false )
    , FirstCall( false )
{}

bool ASerialCommandProcessor::ProcessCommand( void ) {
    while( Serial.available() ) {
        char inputCharacter = (char)Serial.read();
        if( ! ProcessCharacter( inputCharacter ) )
            return false;
    }
    return true;
}

bool ASerialCommandProcessor::ProcessCharacter( char ch ) {
    if( NotRecognized || Rejected )
        return SkipCharacter( ch );

    if( ActiveHandler != NumberOfHandlers )
        return CallHandler( ch );

    if( ch == '\n' ) {
        bool result = ( BodyLength == 0 );
        CommandProcessed();
        return result; // Command was not recognized, nothing to skip
    }

    if( BodyLength == INTERNAL_BODY_LENGTH ) {
        NotRecognized = true;   // Command body buffer is full: command was
        return true;            // not recognized, skip rest of it.
    }

    CommandBody[ BodyLength++ ] = ch;

    for( size_t handlerIndex = 0; handlerIndex < NumberOfHandlers; ++handlerIndex ) {
        if( CommandFound( Handlers[ handlerIndex ].Label ) ) {
            ActiveHandler = handlerIndex;
            FirstCall = true;
            return true;
        }
    }

    return true;
}

bool ASerialCommandProcessor::SkipCharacter( char ch ) {
    if( ch != '\n' )
        return true;

    bool result = ! NotRecognized;
    CommandProcessed();
    return result;
}

bool ASerialCommandProcessor::CallHandler( char ch ) {
    bool callResult = Handlers[ ActiveHandler ].Handler( ch, FirstCall );
    FirstCall = false;

    if( ! callResult ) {
        if( ch != '\n' ) {
            Rejected = true;
            return true;
        }
        // if ch == '\n' there is nothing to reject
    }

    if( ch == '\n' )
        CommandProcessed();

    return true;
}

bool ASerialCommandProcessor::CommandFound( const char* label ) {
    for( size_t i = 0; i < BodyLength; ++i, ++label ) {
        if( CommandBody[ i ] != *label || *label == 0 )
            return false;
    }
    return *label == 0;
}

void ASerialCommandProcessor::CommandProcessed( void ) {
    ActiveHandler = NumberOfHandlers;
    NotRecognized = Rejected = FirstCall = false;
    BodyLength = 0;
}

