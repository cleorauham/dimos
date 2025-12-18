import React from 'react';
import {
  View,
  Text,
  StyleSheet,
  StatusBar,
  SafeAreaView,
} from 'react-native';

const LoadingScreen: React.FC = () => {
  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#1E3A8A" />
      <View style={styles.content}>
        <Text style={styles.logo}>DIMENSIONAL</Text>
        <Text style={styles.version}>v0.0.1</Text>
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1E3A8A', // Blue background
  },
  content: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  logo: {
    fontSize: 32,
    fontWeight: 'bold',
    color: '#FBBF24', // Yellow text
    letterSpacing: 3,
    textAlign: 'center',
    fontFamily: 'monospace', // Gives it a more technical/digital look
  },
  version: {
    fontSize: 12,
    color: '#FBBF24',
    marginTop: 8,
    fontFamily: 'monospace',
  },
});

export default LoadingScreen;
