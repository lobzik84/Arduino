// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

#define DIM(x)              (sizeof(x)/sizeof((x)[0]))

// Command handler has two parameters: a character read from the serial port and a boolean wich
// is true when handler is being caller for the first time for the command.

// If command handler returns false, command is invalid and input stream should be skipped
// until the newline. If command handler returns true it ready for the next character.

struct ACommandHandler {
    const char* Label;
    bool (*Handler)( char ch, bool newCommand );
};

// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

#define INTERNAL_BODY_LENGTH    16

class ASerialCommandProcessor
{
    const ACommandHandler* Handlers;
    const size_t NumberOfHandlers;
    size_t ActiveHandler;

    char CommandBody[ INTERNAL_BODY_LENGTH ];
    size_t BodyLength;

    bool NotRecognized, Rejected, FirstCall;

    bool ProcessCharacter( char );

    bool SkipCharacter( char );
    bool CallHandler( char );
    bool CommandFound( const char* label );
    void CommandProcessed( void );

public:
    ASerialCommandProcessor( const ACommandHandler* handlers, size_t numberOfHandlers );

    // ProcessCommand returns false if command was not recognized
    bool ProcessCommand( void );
}; // class ASerialCommandProcessor

