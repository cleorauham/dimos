import React from 'react';
import {
  View,
  Text,
  StyleSheet,
  StatusBar,
  SafeAreaView,
  TouchableOpacity,
  Alert,
} from 'react-native';

const HomeScreen: React.FC = () => {
  const handleAddRobotDog = () => {
    Alert.alert('Add Robot Dog', 'This feature will be implemented soon!');
  };

  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#1E3A8A" />
      <View style={styles.content}>
        {/* Header with logo */}
        <View style={styles.header}>
          <Text style={styles.logo}>DIMENSIONAL</Text>
          <View style={styles.settingsIcon}>
            <Text style={styles.settingsText}>⚙️</Text>
          </View>
        </View>
        
        {/* Main content area */}
        <View style={styles.mainContent}>
          <TouchableOpacity 
            style={styles.addButton}
            onPress={handleAddRobotDog}
            activeOpacity={0.8}
          >
            <Text style={styles.addButtonText}>Add Robot Dog</Text>
          </TouchableOpacity>
        </View>
        
        {/* Version info */}
        <View style={styles.footer}>
          <Text style={styles.version}>v0.0.1</Text>
        </View>
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
    paddingHorizontal: 20,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingTop: 20,
    paddingBottom: 40,
  },
  logo: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#FBBF24', // Yellow text
    letterSpacing: 2,
    fontFamily: 'monospace',
  },
  settingsIcon: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: '#FBBF24',
    justifyContent: 'center',
    alignItems: 'center',
  },
  settingsText: {
    fontSize: 20,
  },
  mainContent: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  addButton: {
    backgroundColor: '#FBBF24', // Yellow button
    paddingHorizontal: 40,
    paddingVertical: 16,
    borderRadius: 25,
    shadowColor: '#000',
    shadowOffset: {
      width: 0,
      height: 2,
    },
    shadowOpacity: 0.25,
    shadowRadius: 3.84,
    elevation: 5,
  },
  addButtonText: {
    fontSize: 18,
    fontWeight: '600',
    color: '#1E3A8A', // Blue text on yellow button
    textAlign: 'center',
  },
  footer: {
    paddingBottom: 20,
    alignItems: 'center',
  },
  version: {
    fontSize: 12,
    color: '#FBBF24',
    fontFamily: 'monospace',
  },
});

export default HomeScreen;
