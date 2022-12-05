#include "CmdProcessor.h"
#include "RemoteControl.h"

#define RECEIVER_PIN            2
#define LED_PIN                 13
#define MODULATION_FREQUENCY    38

ARemoteControl RemoteControl( RECEIVER_PIN, MODULATION_FREQUENCY );

enum EState {
  EST_IDLE,
  EST_RECORDING,
} State = EST_IDLE;

bool IR_ReportState( char, bool );
bool IR_AbortRecording( char, bool );
bool IR_StartRecording( char, bool );
bool IR_ReplayRecording( char, bool );
bool IR_CCFG( char, bool );
bool IR_PlayRecording( char, bool );

ACommandHandler Commands[] = {
    {   "IR_STATE",     IR_ReportState      },
    {   "IR_ABORTREC",  IR_AbortRecording   },
    {   "IR_RECORD",    IR_StartRecording   },
    {   "IR_REPLAY",    IR_ReplayRecording  },
    {   "IR_CCFG",      IR_CCFG             },
    {   "IR_PLAY",      IR_PlayRecording    },
};

ASerialCommandProcessor CommandProcessor( Commands, DIM( Commands ) );

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

void setup( void ) {
  Serial.begin( 115200 );

  pinMode( LED_PIN, OUTPUT );
  digitalWrite( LED_PIN, LOW );

  pinMode( RECEIVER_PIN, INPUT_PULLUP );
}

void loop( void ) {
    if( ! CommandProcessor.ProcessCommand() )
        Serial.println( "ERROR: unknown command" );

    IR_CompleteRecording();
}

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

bool IR_ReportState( char ch, bool firstCall ) {
    if( firstCall && ch != '\n' ) {
        Serial.println( "IR_ERR: wrong parameter" );
        return false;
    }
    IR_ReportState();
    return true;
}

void IR_ReportState( void ) {
    switch( State ) {
        case EST_IDLE:
            Serial.println( "IR_IDLE" );
            break;
        case EST_RECORDING:
            Serial.println( "IR_RECORDING" );
            break;
        default:
            Serial.println( "IR_UNKNOWN" );
            break;
    }
}

bool IR_AbortRecording( char ch, bool firstCall ) {
    if( firstCall && ch != '\n' ) {
        Serial.println( "IR_ERR: wrong parameter" );
        return false;
    }
    if( State != EST_RECORDING )
        Serial.println( "IR_ERR: not recording" );
    else {
        RemoteControl.StopRecording();
        State = EST_IDLE;
        IR_ReportState();
    }
    return true;
}

bool IR_StartRecording( char ch, bool firstCall ) {
    if( firstCall && ch != '\n' ) {
        Serial.println( "IR_ERR: wrong parameter" );
        return false;
    }
    if( IR_CheckStateIsIdle() ) {
        State = EST_RECORDING;
        RemoteControl.StartRecording();
        IR_ReportState(); // IR_RECORDING
    }
    return true;
}

bool IR_ReplayRecording( char ch, bool firstCall ) {
    if( firstCall && ch != '\n' ) {
        Serial.println( "IR_ERR: wrong parameter" );
        return false;
    }
    IR_ReplayRecording();
    return true;
}

void IR_ReplayRecording( void ) {
    if( IR_CheckStateIsIdle() ) {
        if( RemoteControl.Play() == ARemoteControl::ER_NOTHINGTOPLAY )
            Serial.println( "IR_ERR: no data to play" );
        else
            IR_ReportState(); // IR_IDLE
    }
}

char ParamCCFG[ 8 ];
unsigned short ParamLenCCFG;

bool IR_CCFG( char ch, bool firstCall ) {
    if( firstCall && ch == '\n' ) {
        IR_ReportStateCCFG();
        return true;
    }
    if( firstCall ) {
        if( ch != '=' ) {
            Serial.println( "IR_ERR: wrong parameter" );
            return false;
        }
        ParamLenCCFG = 0;
        return true;
    }
    if( ch == '\n' ) {
        ParamCCFG[ DIM( ParamCCFG ) - 1 ] = 0;
        if( strcmp( ParamCCFG, "ON" ) == 0 ) {
            RemoteControl.SetCCFG( true );
            IR_ReportStateCCFG();
            return true;
        }
        if( strcmp( ParamCCFG, "OFF" ) == 0 ) {
            RemoteControl.SetCCFG( false );
            IR_ReportStateCCFG();
            return true;
        }
        Serial.println( "IR_ERR: wrong parameter" );
        return false;
    }

    if( ParamLenCCFG < DIM( ParamCCFG ) )
        ParamCCFG[ ParamLenCCFG++ ] = ch;

    return true;
}

void IR_ReportStateCCFG( void ) {
  Serial.print( "IR_CCFG=" );
  Serial.println( RemoteControl.GetCCFG() ? "ON" : "OFF" );
}

unsigned long SequenceValue;

bool IR_PlayRecording( char ch, bool firstCall ) {
    if( firstCall ) {
        if( ch != '=' ) {
            Serial.println( "IR_ERR: wrong parameter" );
            return false;
        }
        SequenceValue = 0;
        RemoteControl.StartSequence();
        return true;
    }
    if( ch >= '0' && ch <= '9' ) {
        SequenceValue = SequenceValue * 10 + ch - '0';
        return true;
    }
    else if( ch == ',' || ch == '\n' ) {
        unsigned int value = (unsigned int)SequenceValue;
        if( value != SequenceValue ) {
            Serial.print( "IR_ERR: value oveflow" );
            RemoteControl.StartSequence();
            return false;
        }
        if( ! RemoteControl.AppendToSequence( value ) ) {
            Serial.print( "IR_ERR: sequence buffer overflow" );
            RemoteControl.StartSequence();
            return false;
        }
        SequenceValue = 0;
        if( ch == '\n' ) {
            if( RemoteControl.GetSequenceLength() & 1 == 0 ) {
                Serial.print( "IR_ERR: even number of values in sequence" );
                RemoteControl.StartSequence();
                return false;
            }
            IR_ReplayRecording();
        }
        return true;
    }
    else {
        Serial.print( "IR_ERR: invalid character" );
        RemoteControl.StartSequence();
        return false;
    }
}

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

bool IR_CheckStateIsIdle( void ) {
  if( State == EST_IDLE )
    return true;
  Serial.println( "IR_ERR busy" );
  return false;
}

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

void IR_CompleteRecording( void ) {
  if( State == EST_RECORDING && ! RemoteControl.Recording() ) {
    Serial.print( "IR_CODE=" );
    RemoteControl.PrintCommandSequenceText();
    Serial.write( '\n' );
    State = EST_IDLE;
  }
}

