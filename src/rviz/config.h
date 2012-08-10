/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CONFIG_H
#define CONFIG_H

namespace rviz
{

class ConfigWriter
{
public:
  ConfigWriter();

  void writeToFile( const QString& file_path );
  QString writeToString();

  void keyWithInt( const QString& key_name, int value );
  void keyWithDouble( const QString& key_name, double value );
  void keyWithString( const QString& key_name, const QString& value );
  void keyWithBool( const QString& key_name, bool value );

  /** @brief Emits YAML::Key, key_name, YAML::Value.  Next call should be to emit a value, begin a map, or begin a sequence. */
  void key( const QString& key_name );

  void startMap();
  void endMap();

  void startSeq();
  void endSeq();

  YAML::Emitter& getEmitter();

private:
  YAML::Emitter& emitter_;
};

class ConfigReader
{
public:
  static ConfigReader* openFile( const QString& file_path );
  static ConfigReader* openString( const QString& config_data );

  bool keyWithInt( const QString& key_name, int* value_out );
  bool keyWithDouble( const QString& key_name, double* value_out );
  bool keyWithString( const QString& key_name, QString* value_out );
  bool keyWithBool( const QString& key_name, bool* value_out );

  ConfigReader* keyWithMap( const QString& key_name );
  ConfigSequenceReader* keyWithSeq( const QString& key_name );

  bool isValid();
  QString printableCurrentLocation();
  QString getFilename();
  int getRow();
  int getColumn();
};

class ConfigSequenceReader
{
public:
  bool nextInt( int* value_out );
  bool nextDouble( int* value_out );
  bool nextString( int* value_out );
  bool nextBool( int* value_out );
  ConfigSequenceReader* nextSeq();
  ConfigReader* nextMap();
};

} // end namespace rviz

#endif // CONFIG_H
