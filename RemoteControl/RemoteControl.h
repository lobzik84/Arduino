// -------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------
// --- CPU Frequency -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

#ifndef SYSCLOCK
    #ifdef F_CPU
        #define SYSCLOCK  F_CPU     // main Arduino clock
    #else
        #define SYSCLOCK  16000000  // main Arduino clock
    #endif
#endif

// -------------------------------------------------------------------------------------------------
// --- Timer macro definitions ---------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

#if defined( __AVR_ATmega2560__ )

    // Use TIMER2 (8 bits) for pin 9
    #define TIMER_PWM_PIN       9

    #define TIMER_ENABLE_PWM    ( TCCR2A |= _BV(COM2B1) )
    #define TIMER_DISABLE_PWM   ( TCCR2A &= ~(_BV(COM2B1)) )
    #define TIMER_ENABLE_INTR   ( TIMSK2 = _BV(OCIE2A) )
    #define TIMER_DISABLE_INTR  ( TIMSK2 = 0 )
    #define TIMER_PWM_PIN  9              // Arduino Mega

    #define TIMER_CONFIG_KHZ(val) ({ \
        const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
        TCCR2A               = _BV(WGM20); \
        TCCR2B               = _BV(WGM22) | _BV(CS20); \
        OCR2A                = pwmval; \
        OCR2B                = pwmval / 3; \
    })

    #define SYSCLOCK  F_CPU     // main Arduino clock

#elif defined( __AVR_ATmega328__ ) || defined( __AVR_ATmega328P__ )

    // Use TIMER1 (16 bits) for pin 9
    #define TIMER_PWM_PIN       9

    #define TIMER_ENABLE_PWM   (TCCR1A |= _BV(COM1A1))
    #define TIMER_DISABLE_PWM  (TCCR1A &= ~(_BV(COM1A1)))

    #if defined(__AVR_ATmega8P__) || defined(__AVR_ATmega8__)
        #define TIMER_ENABLE_INTR   (TIMSK |= _BV(OCIE1A))
        #define TIMER_DISABLE_INTR  (TIMSK &= ~_BV(OCIE1A))
    #else
        #define TIMER_ENABLE_INTR   (TIMSK1 = _BV(OCIE1A))
        #define TIMER_DISABLE_INTR  (TIMSK1 = 0)
    #endif

    #define TIMER_CONFIG_KHZ(val) ({ \
        const uint16_t pwmval = SYSCLOCK / 2000 / (val); \
        TCCR1A                = _BV(WGM11); \
        TCCR1B                = _BV(WGM13) | _BV(CS10); \
        ICR1                  = pwmval; \
        OCR1A                 = pwmval / 3; \
    })

#else

    #error Unsupported AVR card found. Please modify RemoteControl.h

#endif

// -------------------------------------------------------------------------------------------------
// --- class ARemoteControl ------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

#define COMMAND_SEQUENCE_BUFFER_SIZE            330
class ARemoteControl {
public:
    enum EResult {
        ER_SUCCESS = 0,
        ER_EVENCOUNT,
        ER_WRONGCHAR,
        ER_BUFFEROVERFLOW,
        ER_NOTHINGTOPLAY,
    };

private:
    const unsigned int ReceiverPin;
    const unsigned int ModulationFrequency;

    unsigned int CommandSequence[ COMMAND_SEQUENCE_BUFFER_SIZE ];
    unsigned int CommandSequenceIndex;

    unsigned long LastCounter;
    bool LastCounterHasValue;
    bool StopRecordOnTimer;
    bool StateCCFG;

    static void CustomDelay( unsigned int uSecs );
    static void OnChangeIR( void );
    void ProcessChangeIR( void );

    void StartTransmitter( void );

public:
    ARemoteControl( unsigned int receiverPin, unsigned int modulFreq );

    // --- Stored sequence methods
    unsigned int GetSequenceLength( void ) const {
        return CommandSequenceIndex;
    }
    void StartSequence( void ) {
        CommandSequenceIndex = 0;
    }
    bool AppendToSequence( unsigned int );
    void PrintCommandSequenceText( void ) const;

    // --- Recording methods
    void StartRecording( void );
    bool Recording( void ) const;
    void StopRecording( bool onTimerOnly = false );

    // --- CCFG methods
    void SetCCFG( bool );
    bool GetCCFG( void ) const {
        return StateCCFG;
    }

    // Methods for playing command sequence
    EResult Play( void );
}; // class ARemoteControl

