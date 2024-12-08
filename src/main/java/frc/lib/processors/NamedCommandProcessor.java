package frc.lib.processors;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.annotations.NamedCommand;
import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.util.HashMap;
import java.util.Map;

public class NamedCommandProcessor {

  private static final Map<Class<?>, Object> dependencyMap = new HashMap<>();

  public static void processClass(Class<?> clazz) {
    // Iterate over all methods in the class
    for (Method method : clazz.getDeclaredMethods()) {
      // Check if the method is annotated with @NamedCommand
      if (method.isAnnotationPresent(NamedCommand.class)) {
        NamedCommand annotation = method.getAnnotation(NamedCommand.class);

        // Verify the return type is a Command
        if (!Command.class.isAssignableFrom(method.getReturnType())) {
          throw new IllegalArgumentException(
              "Method "
                  + method.getName()
                  + " annotated with @NamedCommand must return a Command.");
        }

        try {
          // Make the method accessible if it's private
          if (!method.canAccess(null)) {
            method.setAccessible(true);
          }

          // Prepare the parameters for the method
          Object[] parameters = prepareParameters(method);

          // Call the method (assumes it's static)
          Command command = (Command) method.invoke(null, parameters);

          // Register the command using NamedCommands
          NamedCommands.registerCommand(annotation.value(), command);

          System.out.println("Registered command " + annotation.value());

        } catch (Exception e) {
          throw new RuntimeException(
              "Failed to process method " + method.getName() + " in class " + clazz.getName(), e);
        }
      }
    }
  }

  private static Object[] prepareParameters(Method method) {
    Parameter[] parameters = method.getParameters();
    Object[] parameterValues = new Object[parameters.length];

    for (int i = 0; i < parameters.length; i++) {
      Parameter parameter = parameters[i];
      Class<?> parameterType = parameter.getType();
      Object dependency = dependencyMap.get(parameterType);
      if (dependency == null) {
        throw new IllegalArgumentException(
            "No dependency found for type " + parameterType.getName());
      }
      parameterValues[i] = dependency;
    }

    return parameterValues;
  }

  public static void registerDependency(Class<?> type, Object instance) {
    dependencyMap.put(type, instance);
  }
}
