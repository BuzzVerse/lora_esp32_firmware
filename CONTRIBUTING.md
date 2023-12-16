# Contributing Guidelines

Thank you for considering contributing to our project! We appreciate your interest in helping us improve our codebase. To ensure a smooth and efficient collaboration, please follow the guidelines below when documenting code using the Doxygen format.

## Code Documentation with Doxygen

### What is Doxygen?

[Doxygen](https://www.doxygen.nl/) is a documentation generator that can be used to automatically generate documentation from annotated source code. It supports various programming languages and provides a standardized way to document code, making it easier for developers to understand, maintain, and contribute to the project.

### Documenting Code

1. **Function and Class Documentation:**
   - Use Doxygen comments to document all functions, methods, and classes.
   - Provide a brief description of the purpose and functionality of the function or class.
   - Include details about input parameters, output values, and any exceptions that may be thrown.

   ```cpp
   /**
    * @brief Brief description of the function or method.
    *
    * Detailed description (if necessary).
    *
    * @param param1 Description of parameter 1.
    * @param param2 Description of parameter 2.
    * @return Description of the return value.
    * @throw ExceptionType Description of the exception (if applicable).
    */
   int exampleFunction(int param1, int param2);
   ```

2. **File-Level Documentation:**
   - Provide a brief overview of the purpose and contents of each file.
   - Mention any important conventions, data structures, or algorithms used in the file.

   ```c
      /**
      * @file filename.c
      * @brief Brief description of the file.
      *
      * Detailed description (if necessary).
      */
   ```

3. **Grouping and Modules:**
   - Use Doxygen grouping to organize related functions, classes, and modules.
   - Clearly define and document the purpose of each group or module.

   ```cpp
   /**
    * @defgroup ModuleName Module Description
    * @{
    */

   // ... Code related to the module ...

   /**
    * @}
    */
   ```

### Building Documentation

- Ensure that Doxygen is installed on your local machine. You can download it [here](https://www.doxygen.nl/download.html).
- Run Doxygen on the project to generate the documentation, and make sure that the output is correct.

   ```bash
   doxygen Doxyfile
   ```

### Additional Resources

- [Doxygen Documentation](https://www.doxygen.nl/manual/index.html)

## Conclusion

By following these guidelines, you contribute not only to the codebase but also to the overall understanding and maintainability of the project. Thank you for your commitment to quality documentation! If you have any questions or need assistance, feel free to reach out to the maintainers or community. Happy coding!
