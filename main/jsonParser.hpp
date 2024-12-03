/**
 * @file jsonParser.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <string>
#include <string.h>
#include <ctype.h>
#include <unordered_map>

using JSONDict = std::unordered_map<std::string, std::string>;

/**
 * @brief Parses a json into a map of strings
 * 
 * @param[in] json null-terminated input string in JSON format
 * @param[in] str_len number of characters in the JSON string
 * @param[out] result hash-map containing elements of the JSON file
 * @return Number of new elements stored in result
 */
inline int parse_JSON(
	const char *json,
	const int str_len,
	JSONDict &result
)
{
	char key_buffer[BUFSIZ] = {0};
	char val_buffer[BUFSIZ] = {0};
	int key_buf_idx = 0;
	int val_buf_idx = 0;
	int idx = 0;
	int n_brackets = 0;
	int n_elements = 0;

	// skips initial whitespace and starting '{'
	while (isblank(json[idx]) || json[idx]=='{')
		idx++;
	
	enum curr_process {
		BEFORE_KEY,
		KEY,
		BEFORE_SEPARATOR,
		AFTER_SEPARATOR,
		VALUE,
		AFTER_VALUE
	};
	enum val_delimitator {
		QUOTATION,
		BRACKETS,
		NONE,
	};
	curr_process processing = BEFORE_KEY;
	val_delimitator delim = NONE;
	while (idx < str_len && json[idx]!='\0') {
		switch (processing) {
			case BEFORE_KEY:
				if (json[idx]=='}') {
					return n_elements;
				}
				if (json[idx]=='"') processing = KEY;
				break;
			case KEY:
				if (json[idx]=='"') {
					processing = BEFORE_SEPARATOR;
					key_buffer[key_buf_idx] = '\0';
					break;
				}
				key_buffer[key_buf_idx++] = json[idx];
				break;
			case BEFORE_SEPARATOR:
				if (json[idx]==':') processing = AFTER_SEPARATOR;
				break;
			case AFTER_SEPARATOR:
				if (!isblank(json[idx])) {
					processing = VALUE;
					if (json[idx]=='"') {
						delim = QUOTATION;
					}
					else if (json[idx]=='{'||json[idx]=='[') {
						delim = BRACKETS;
						n_brackets = 1;
						val_buffer[val_buf_idx++] = json[idx];
					}
					else {
						delim = NONE;
						val_buffer[val_buf_idx++] = json[idx];
					}
				}
				break;
			case VALUE:
				if (
					(json[idx]=='"' && delim==QUOTATION)
					||
					(isblank(json[idx] && delim==NONE))
				) {
					processing = AFTER_VALUE;
					val_buffer[val_buf_idx] = '\0';
					result[key_buffer] = val_buffer;
					key_buf_idx = 0;
					val_buf_idx = 0;
					n_elements++;
					break;
				}
				if (json[idx]==',' && delim==NONE) {
					processing = BEFORE_KEY;
					val_buffer[val_buf_idx] = '\0';
					result[key_buffer] = val_buffer;
					key_buf_idx = 0;
					val_buf_idx = 0;
					n_elements++;
					break;
				}
				val_buffer[val_buf_idx++] = json[idx];
				if (json[idx]=='{'||json[idx]=='[')
					n_brackets++;
				if (json[idx]=='}'||json[idx]==']') {
					if(--n_brackets == 0) {
						processing = AFTER_VALUE;
						val_buffer[val_buf_idx] = '\0';
						result[key_buffer] = val_buffer;
						key_buf_idx = 0;
						val_buf_idx = 0;
						n_elements++;
					}
				}
				break;
			case AFTER_VALUE:
				if (json[idx]==','||json[idx]=='}') {
					processing = BEFORE_KEY;
					if (json[idx]=='}') {
						return n_elements;
					}
				}
			break;
		}
		idx++;
	}

	return n_elements;
}