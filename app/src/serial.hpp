#include <cstring>
#include <string_view>
#include <charconv>

static const uint32_t MAX_MSG_LEN = 64;
static const uint32_t MAX_PARAMS = 4;

enum CommandType {
    Calibrate = 0,
    Position = 1,
    InvalidCmd,
};

class Message {
    CommandType mType;
    int32_t mParams[MAX_PARAMS];
    uint32_t mSize;

public:
    Message() :
        mType(CommandType::InvalidCmd),
        mSize(0)
    {
    }

    Message(CommandType type) :
        mType(type),
        mSize(0)
    {       
    }

    CommandType type() { return mType; };

    int32_t param_int(uint32_t idx) {
        if(idx >= mSize) {
            return 0;
        } else {
            return mParams[idx];
        }
    }

    float param_float(uint32_t idx) {
        if(idx >= mSize) {
            return 0.0;
        } else {
            return *((float *)&mParams[idx]);
        }
    }

    void push_param(int32_t value) {
        if (mSize < MAX_PARAMS) {
            mParams[mSize] = value;
            mSize += 1;
        }
    }

    void push_param(float value) {
        if(mSize < MAX_PARAMS) {
            mParams[mSize] = *((int32_t*)&value);
        }
    }
};

class CmdTokenizer {
public:
    CmdTokenizer(char *buffer, size_t len) :
        mFullString(buffer, len)
    {
        mWordCount = 0;
        bool blank = true;
        for(uint32_t i=0; i<mFullString.size(); i++) {
            if(blank) {
                if(is_blank(mFullString[i])) {
                    continue;
                } else {
                    blank = false;
                    mWordCount += 1;
                }
            } else {
                if(is_blank(mFullString[i])) {
                    blank = true;
                }
            }
        }
    }

    std::string_view word(size_t n) {
        if(n >= mWordCount) {
            return std::string_view();
        }

        size_t word_pos = 0;
        size_t start = 0;
        bool blank = true;
        for(uint32_t i=0; i<mFullString.size(); i++) {
            if(blank) {
                if(is_blank(mFullString[i])) {
                    continue;
                } else {
                    if(word_pos == n) {
                        start = i;
                    }

                    blank = false;
                }
            } else {
                if(is_blank(mFullString[i])) {
                    if(word_pos == n) {
                        return mFullString.substr(start, i - start);
                    } else {
                        word_pos += 1;
                    }
                    blank = true;
                }
            }
        }
        return mFullString.substr(start);
    }
    
    size_t words() {
        return mWordCount;
    }

private:
    std::string_view mFullString;
    size_t mWordCount;

    bool is_blank(char c) {
        return c == ' ' || c == '\r' || c == '\n';
    }
};

class SerialProcessing {
    uint8_t buffer[MAX_MSG_LEN];
    uint32_t ptr = 0;

public:
    bool push_byte(uint8_t b, Message &out) {
        if(b == '\n') {
            // std::string_view s((char *)buffer);
            // size_t first_space = s.find_first_of(" ");
            // if(first_space == std::string_view::npos) {
            //     first_space = s.size();
            // }
            // std::string_view cmd = s.substr(0, first_space);
            
            CmdTokenizer tokens((char *)buffer, ptr);
            ptr = 0;
            if(tokens.words() == 0) {
                return false;
            }
            std::string_view cmd = tokens.word(0);
            if(cmd.compare("CAL") == 0) {
                out = Message(CommandType::Calibrate);
                return true;
            } else if(cmd.compare("P") == 0) {
                if(tokens.words() < 2) {
                    return false;
                }
                out = Message(CommandType::Position);
                std::string_view pos_str = tokens.word(1);
                int32_t pos = 180;
                std::from_chars_result result = std::from_chars(pos_str.data(), pos_str.data() + pos_str.size(), pos);
                out.push_param(pos);
                return true;
            }
            else {
                return false;
            }
        } else {
            if(ptr < MAX_MSG_LEN - 1) {
                buffer[ptr] = b;
                ptr += 1;
            } else {
                ptr = 0; // Too long. Abort message.
            }
            return false;
        }
    }
};
