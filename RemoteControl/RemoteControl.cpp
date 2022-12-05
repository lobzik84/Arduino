#include <Arduino.h>
#include "RemoteControl.h"

// preload timer 65536-16MHz/256/2Hz
#define RECORD_TIMER_PRELOAD_VALUE      34286
#define MAX_INTERVAL 15000 //us

ARemoteControl* RemoteControlSingletonImpl = NULL;

ARemoteControl::ARemoteControl( unsigned int receiverPin, unsigned int modulFreq )
    : ReceiverPin( receiverPin )
    , StateCCFG( false )
    , ModulationFrequency( modulFreq )
{
    attachInterrupt( digitalPinToInterrupt( ReceiverPin ), OnChangeIR, CHANGE );
}

void ARemoteControl::OnChangeIR( void ) {
    if( RemoteControlSingletonImpl )
        RemoteControlSingletonImpl->ProcessChangeIR();
}

void ARemoteControl::ProcessChangeIR( void ) {
    const unsigned long counter = micros();
    unsigned long value = counter - LastCounter;
    LastCounter = counter;

    if( ! LastCounterHasValue ) {
        LastCounterHasValue = true;
        return;
    }

    if( CommandSequenceIndex == COMMAND_SEQUENCE_BUFFER_SIZE || value > MAX_INTERVAL ) {
        StopRecording();
        return;
    }

    if( AppendToSequence( value ) )
        TCNT1 = RECORD_TIMER_PRELOAD_VALUE;

    StopRecordOnTimer = ( digitalRead( ReceiverPin ) == HIGH );
}

bool ARemoteControl::Recording( void ) const {
    return RemoteControlSingletonImpl == this;
}

void ARemoteControl::StartRecording( void ) {
    SetCCFG( false );
    noInterrupts();
    RemoteControlSingletonImpl = this;

    StartSequence();
    StopRecordOnTimer = false;

    LastCounter = 0;
    LastCounterHasValue = false;

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = RECORD_TIMER_PRELOAD_VALUE;
    TCCR1B |= (1 << CS12);    // 256 prescaler
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();
}

void ARemoteControl::StopRecording( bool onTimerOnly ) {
    if( StopRecordOnTimer || ! onTimerOnly ) {
        if( RemoteControlSingletonImpl == this ) {
            TIMSK1 &= ~( 1 << TOIE1 );  // disable timer overflow interrupt
            RemoteControlSingletonImpl = NULL;
        }
    }
}

bool ARemoteControl::AppendToSequence( unsigned int value ) {
    if( CommandSequenceIndex == COMMAND_SEQUENCE_BUFFER_SIZE )
        return false;
    CommandSequence[ CommandSequenceIndex++ ] = value;
    return true;
}

void ARemoteControl::PrintCommandSequenceText( void ) const {
    if( CommandSequenceIndex > 0 ) {
      Serial.print( CommandSequence[ 0 ], DEC );
      for( unsigned int i = 1; i < CommandSequenceIndex; ++i ) {
          Serial.write( ',' );
          Serial.print( CommandSequence[ i ], DEC );
      }
    }
}

void ARemoteControl::SetCCFG( bool ccfg ) {
    if( ccfg != StateCCFG ) {
        if( ccfg ) {
            delay( 50 );
            StartTransmitter();
            TIMER_ENABLE_PWM;
        }
        else {
            TIMER_DISABLE_PWM;
            delay( 50 );
        }
        StateCCFG = ccfg;
    }
}

void ARemoteControl::StartTransmitter( void ) {
    // Disable the Timer2 Interrupt (which is used for receiving IR)
    TIMER_DISABLE_INTR; //Timer2 Overflow Interrupt

    pinMode( TIMER_PWM_PIN, OUTPUT );
    digitalWrite( TIMER_PWM_PIN, LOW ); // When not sending PWM, we want it low

    // COM2A = 00: disconnect OC2A
    // COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
    // WGM2 = 101: phase-correct PWM with OCRA as top
    // CS2  = 000: no prescaling
    // The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
    TIMER_CONFIG_KHZ( ModulationFrequency );
}

void ARemoteControl::CustomDelay( unsigned int uSecs ) {
    if( uSecs > 4 ) {
        unsigned long start = micros();
        unsigned long endMicros = start + uSecs - 4;
        if( endMicros < start ) { // Check if overflow
            while( micros() > start ) {} // wait until overflow
        }
        while( micros() < endMicros ) {} // normal wait
    }
}

ARemoteControl::EResult ARemoteControl::Play( void ) {
    if( CommandSequenceIndex == 0 )
        return ER_NOTHINGTOPLAY;
    SetCCFG( false );
    StartTransmitter();
    for( unsigned int i = 0; i < CommandSequenceIndex; ++i ) {
        if( i & 1 ) {
            TIMER_DISABLE_PWM; // show space
        }
        else {
            TIMER_ENABLE_PWM; // show mark
        }
        CustomDelay( CommandSequence[ i ] );
    }
    TIMER_DISABLE_PWM; // Always end with the LED off
    delay( 50 );
    return ER_SUCCESS;
}

ISR( TIMER1_OVF_vect ) {
    if( RemoteControlSingletonImpl )
        RemoteControlSingletonImpl->StopRecording( true );
}

