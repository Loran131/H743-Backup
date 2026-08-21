/*
 * MIT License
 *
 * Copyright (c) 2010 Serge Zaitsev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef JSMN_H
#define JSMN_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    JSMN_UNDEFINED = 0,
    JSMN_OBJECT = 1 << 0,
    JSMN_ARRAY = 1 << 1,
    JSMN_STRING = 1 << 2,
    JSMN_PRIMITIVE = 1 << 3
} jsmntype_t;

enum jsmnerr {
    JSMN_ERROR_NOMEM = -1,
    JSMN_ERROR_INVAL = -2,
    JSMN_ERROR_PART = -3
};

typedef struct jsmntok {
    jsmntype_t type;
    int start;
    int end;
    int size;
} jsmntok_t;

typedef struct jsmn_parser {
    unsigned int pos;
    unsigned int toknext;
    int toksuper;
} jsmn_parser;

static jsmntok_t *jsmn_alloc_token(jsmn_parser *parser, jsmntok_t *tokens,
                                   size_t count)
{
    jsmntok_t *token;
    if (parser->toknext >= count) return NULL;
    token = &tokens[parser->toknext++];
    token->start = -1;
    token->end = -1;
    token->size = 0;
    token->type = JSMN_UNDEFINED;
    return token;
}

static void jsmn_fill_token(jsmntok_t *token, jsmntype_t type,
                            int start, int end)
{
    token->type = type;
    token->start = start;
    token->end = end;
    token->size = 0;
}

static int jsmn_parse_primitive(jsmn_parser *parser, const char *json,
                                size_t length, jsmntok_t *tokens,
                                size_t count)
{
    int start = (int)parser->pos;
    jsmntok_t *token;
    for (; parser->pos < length; ++parser->pos) {
        char c = json[parser->pos];
        if ((c == '\t') || (c == '\r') || (c == '\n') || (c == ' ') ||
            (c == ',') || (c == ']') || (c == '}')) break;
        if ((unsigned char)c < 32U) {
            parser->pos = (unsigned int)start;
            return JSMN_ERROR_INVAL;
        }
    }
    token = jsmn_alloc_token(parser, tokens, count);
    if (token == NULL) {
        parser->pos = (unsigned int)start;
        return JSMN_ERROR_NOMEM;
    }
    jsmn_fill_token(token, JSMN_PRIMITIVE, start, (int)parser->pos);
    if (parser->pos != 0U) --parser->pos;
    return 0;
}

static int jsmn_parse_string(jsmn_parser *parser, const char *json,
                             size_t length, jsmntok_t *tokens, size_t count)
{
    int start = (int)parser->pos;
    jsmntok_t *token;
    ++parser->pos;
    for (; parser->pos < length; ++parser->pos) {
        char c = json[parser->pos];
        if (c == '"') {
            token = jsmn_alloc_token(parser, tokens, count);
            if (token == NULL) {
                parser->pos = (unsigned int)start;
                return JSMN_ERROR_NOMEM;
            }
            jsmn_fill_token(token, JSMN_STRING, start + 1,
                            (int)parser->pos);
            return 0;
        }
        if ((c == '\\') && (parser->pos + 1U < length)) {
            char escaped = json[++parser->pos];
            if (escaped == 'u') {
                for (uint8_t i = 0U; i < 4U; ++i) {
                    char hex;
                    if (++parser->pos >= length) return JSMN_ERROR_PART;
                    hex = json[parser->pos];
                    if (!(((hex >= '0') && (hex <= '9')) ||
                          ((hex >= 'A') && (hex <= 'F')) ||
                          ((hex >= 'a') && (hex <= 'f'))))
                        return JSMN_ERROR_INVAL;
                }
            } else if ((escaped != '"') && (escaped != '/') &&
                       (escaped != '\\') && (escaped != 'b') &&
                       (escaped != 'f') && (escaped != 'r') &&
                       (escaped != 'n') && (escaped != 't')) {
                return JSMN_ERROR_INVAL;
            }
        }
    }
    parser->pos = (unsigned int)start;
    return JSMN_ERROR_PART;
}

static void jsmn_init(jsmn_parser *parser)
{
    parser->pos = 0U;
    parser->toknext = 0U;
    parser->toksuper = -1;
}

static int jsmn_parse(jsmn_parser *parser, const char *json, size_t length,
                      jsmntok_t *tokens, unsigned int count)
{
    int found = 0;
    for (; parser->pos < length; ++parser->pos) {
        char c = json[parser->pos];
        jsmntok_t *token;
        int result;
        int i;
        switch (c) {
        case '{':
        case '[':
            token = jsmn_alloc_token(parser, tokens, count);
            if (token == NULL) return JSMN_ERROR_NOMEM;
            if (parser->toksuper >= 0) tokens[parser->toksuper].size++;
            token->type = (c == '{') ? JSMN_OBJECT : JSMN_ARRAY;
            token->start = (int)parser->pos;
            parser->toksuper = (int)parser->toknext - 1;
            ++found;
            break;
        case '}':
        case ']': {
            jsmntype_t type = (c == '}') ? JSMN_OBJECT : JSMN_ARRAY;
            for (i = (int)parser->toknext - 1; i >= 0; --i) {
                token = &tokens[i];
                if ((token->start != -1) && (token->end == -1)) {
                    if (token->type != type) return JSMN_ERROR_INVAL;
                    token->end = (int)parser->pos + 1;
                    parser->toksuper = -1;
                    break;
                }
            }
            if (i < 0) return JSMN_ERROR_INVAL;
            for (--i; i >= 0; --i) {
                if ((tokens[i].start != -1) && (tokens[i].end == -1)) {
                    parser->toksuper = i;
                    break;
                }
            }
            break;
        }
        case '"':
            result = jsmn_parse_string(parser, json, length, tokens, count);
            if (result < 0) return result;
            ++found;
            if (parser->toksuper >= 0) tokens[parser->toksuper].size++;
            break;
        case '\t': case '\r': case '\n': case ' ':
            break;
        case ':':
            parser->toksuper = (int)parser->toknext - 1;
            break;
        case ',':
            if ((parser->toksuper >= 0) &&
                (tokens[parser->toksuper].type != JSMN_ARRAY) &&
                (tokens[parser->toksuper].type != JSMN_OBJECT)) {
                for (i = (int)parser->toknext - 1; i >= 0; --i) {
                    if (((tokens[i].type == JSMN_ARRAY) ||
                         (tokens[i].type == JSMN_OBJECT)) &&
                        (tokens[i].end == -1)) {
                        parser->toksuper = i;
                        break;
                    }
                }
            }
            break;
        default:
            result = jsmn_parse_primitive(parser, json, length,
                                          tokens, count);
            if (result < 0) return result;
            ++found;
            if (parser->toksuper >= 0) tokens[parser->toksuper].size++;
            break;
        }
    }
    for (int i = (int)parser->toknext - 1; i >= 0; --i) {
        if ((tokens[i].start != -1) && (tokens[i].end == -1))
            return JSMN_ERROR_PART;
    }
    return found;
}

#endif
