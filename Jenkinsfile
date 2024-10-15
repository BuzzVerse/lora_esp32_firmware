// Place method definitions here, outside the pipeline block

// Function to read the number of errors from the CSV file
def getErrorsFromCsv(csvContent) {
    // Normalize line endings to \n
    csvContent = csvContent.replaceAll('\r\n', '\n').replaceAll('\r', '\n')

    // Split lines and remove empty ones
    def lines = csvContent.trim().split('\n').findAll { it.trim() }

    if (lines.size() < 2) {
        error "Invalid CSV file format: Expected at least 2 lines, got ${lines.size()}"
    }

    // Split the header fields
    def headerFields = lines[0].split(',').collect { it.trim() }

    // Check if the header contains 'Errors'
    def errorsIndex = headerFields.findIndexOf { it.equalsIgnoreCase('Errors') }
    if (errorsIndex != -1) {
        // Format where 'Errors' is in the header
        def dataFields = lines[1].split(',').collect { it.trim() }
        def errorsStr = dataFields[errorsIndex]

        // Check if errorsStr is a valid integer
        if (!errorsStr.isInteger()) {
            error "Invalid error count: '${errorsStr}' is not an integer"
        }

        return errorsStr.toInteger()
    } else {
        // Format where 'Type,Count' is the header
        def typeIndex = headerFields.findIndexOf { it.equalsIgnoreCase('Type') }
        def countIndex = headerFields.findIndexOf { it.equalsIgnoreCase('Count') }

        if (typeIndex == -1 || countIndex == -1) {
            error "CSV file does not contain expected headers 'Type' and 'Count'"
        }

        // Find the row where 'Type' is 'Errors'
        def errorsStr = null
        for (int i = 1; i < lines.size(); i++) {
            def dataFields = lines[i].split(',').collect { it.trim() }
            if (dataFields[typeIndex].equalsIgnoreCase('Errors')) {
                errorsStr = dataFields[countIndex]
                break
            }
        }

        if (errorsStr == null) {
            error "Could not find 'Errors' row in CSV file"
        }

        // Check if errorsStr is a valid integer
        if (!errorsStr.isInteger()) {
            error "Invalid error count: '${errorsStr}' is not an integer"
        }

        return errorsStr.toInteger()
    }
}

// Function to get the branch name from a build
@NonCPS
def getBuildBranch(build) {
    def buildData = build.getAction(hudson.plugins.git.util.BuildData)
    if (buildData == null) {
        return null
    }
    def lastBuiltRevision = buildData.lastBuiltRevision
    if (lastBuiltRevision == null) {
        return null
    }
    def branches = lastBuiltRevision.branches
    if (branches == null || branches.isEmpty()) {
        return null
    }
    return branches.iterator().next().name
}

// Function to find the reference build number
@NonCPS
def findReferenceBuildNumber(currentBuild) {
    def build = currentBuild.rawBuild.getPreviousBuild() // Start from the previous build
    while (build != null) {
        def buildBranch = getBuildBranch(build)
        if (buildBranch == null) {
            build = build.getPreviousBuild()
            continue
        }

        // Normalize the branch name
        def normalizedBranchName = buildBranch.replaceFirst(/^refs\/(heads|remotes\/origin)\//, '').replaceFirst(/^origin\//, '')

        if (normalizedBranchName == 'jenkins_test') {
            if (build.getResult() == null || build.getResult().toString() == 'SUCCESS' || build.getResult().toString() == 'UNSTABLE') {
                return build.number.toString()
            }
        }
        build = build.getPreviousBuild()
    }
    return null
}

pipeline {
    agent any
    environment {
        IDF_PATH = "/opt/esp/idf"  // Path to ESP-IDF
    }
    parameters {
        string(name: 'GIT_BRANCH', defaultValue: 'jenkins_test', description: 'Branch to build from')
    }
    stages {
        stage('Checkout') {
            steps {
                echo "Checking out code from branch: ${params.GIT_BRANCH}"
                checkout([
                    $class: 'GitSCM',
                    branches: [[name: "${params.GIT_BRANCH}"]],
                    userRemoteConfigs: [[
                        url: 'https://github.com/BuzzVerse/lora_esp32_firmware.git',
                        credentialsId: 'jenkins1'
                    ]],
                    extensions: [
                        [$class: 'SubmoduleOption',
                            recursiveSubmodules: true,
                            trackingSubmodules: true,
                            parentCredentials: true,
                            timeout: 10
                        ]]
                ])
                echo "Checkout completed."
            }
        }

        stage('Build with ESP-IDF') {
            steps {
                script {
                    echo "Starting build with ESP-IDF..."
                    // Activating the ESP-IDF environment and building the project
                    def buildStatus = sh(
                        script: '''
                        . /opt/esp/idf/export.sh  # Activate ESP-IDF environment
                        idf.py build  # Build the project
                        ''',
                        returnStatus: true  // Allow pipeline continuation even if there is a failure
                    )

                    if (buildStatus != 0) {
                        echo "Build failed, but the pipeline will continue"
                        // Additional logic can be added here
                    } else {
                        echo "Build succeeded."
                    }
                }
            }
        }

        stage('Run Cppcheck') {
            steps {
                script {
                    echo "Running Cppcheck analysis..."

                    // Filtering compile_commands.json to exclude unwanted directories
                    sh '''
                        jq 'map(select(.file | test("^(/opt/esp/idf/|build/)") | not))' build/compile_commands.json > build/compile_commands_filtered.json
                    '''

                    // Running C/C++ code analysis using cppcheck and generating an XML report
                    sh '''
                        cppcheck --enable=all --inconclusive --force \
                        --suppress=missingIncludeSystem \
                        --project=build/compile_commands_filtered.json --xml --xml-version=2 \
                        2> cppcheck-report.xml || true
                    '''

                    // Verify that cppcheck-report.xml is not empty
                    sh '''
                        if [ ! -s cppcheck-report.xml ]; then
                            echo "cppcheck-report.xml is empty."
                            exit 1
                        fi
                    '''

                    // Generating an HTML report using cppcheck-htmlreport
                    sh '''
                        cppcheck-htmlreport --file=cppcheck-report.xml --report-dir=cppcheck-html --source-dir=. --title="Cppcheck Analysis Report"
                    '''

                    // Extracting the number of errors from the XML report and creating a CSV file
                    sh '''
                        ERRORS=$(grep "<error " cppcheck-report.xml | wc -l)
                        echo "Type,Count" > cppcheck-results.csv
                        echo "Errors,$ERRORS" >> cppcheck-results.csv
                    '''
                    echo "Cppcheck analysis completed. Report generated."
                }
            }
        }

        stage('Compare with Reference Build') {
            steps {
                script {
                    echo "Comparing with reference build..."
                    // Read the number of errors from the current build
                    def currCsvContent = readFile('cppcheck-results.csv')
                    def currErrors = getErrorsFromCsv(currCsvContent)

                    // Find the reference build number using an @NonCPS method
                    def refBuildNumber = findReferenceBuildNumber(currentBuild)

                    if (refBuildNumber != null) {
                        echo "Reference build found: #${refBuildNumber} on branch 'jenkins_test'"

                        // Copy artifact from the reference build
                        copyArtifacts(
                            projectName: env.JOB_NAME,
                            selector: [$class: 'SpecificBuildSelector', buildNumber: "${refBuildNumber}"],
                            filter: 'cppcheck-results.csv',
                            target: 'reference',
                            optional: false
                        )

                        // Ensure the file exists after copying
                        if (!fileExists('reference/cppcheck-results.csv')) {
                            error "cppcheck-results.csv file not found in 'reference' directory after copying."
                        }

                        // Read the number of errors from the reference build
                        def refCsvContent = readFile('reference/cppcheck-results.csv')
                        def refErrors = getErrorsFromCsv(refCsvContent)

                        echo "Comparing error counts: Current build errors = ${currErrors}, Reference build errors = ${refErrors}"

                        if (params.GIT_BRANCH == 'jenkins_test') {
                            // For 'jenkins_test' branch, compare with previous build on this branch
                            if (currErrors > refErrors) {
                                error "Current build has more errors (${currErrors}) than the previous build (${refErrors}) on branch 'jenkins_test'."
                            } else {
                                echo "Current build does not have more errors than the previous build on branch 'jenkins_test'."
                            }
                        } else {
                            // For other branches, ensure errors are less than or equal to 'jenkins_test'
                            if (currErrors > refErrors) {
                                error "Current build has more errors (${currErrors}) than the reference build (${refErrors}) on branch 'jenkins_test'."
                            } else {
                                echo "Current build has an acceptable number of errors compared to 'jenkins_test' (current: ${currErrors}, reference: ${refErrors})."
                            }
                        }
                    } else {
                        error "No reference build found on branch 'jenkins_test' to compare with."
                    }
                }
            }
        }

        stage('Plot Cppcheck Results') {
            when {
                expression {
                    return params.GIT_BRANCH == 'jenkins_test'  // Execute this stage only for 'jenkins_test' branch
                }
            }
            steps {
                echo "Plotting Cppcheck results..."
                plot csvFileName: 'cppcheck-results.csv',
                     group: 'Cppcheck Analysis',
                     title: 'Cppcheck Errors Over Time',
                     style: 'line',
                     yaxis: 'Number of Errors',
                     csvSeries: [
                         [file: 'cppcheck-results.csv', inclusionFlag: 'OFF', displayTableFlag: false, label: 'Errors']
                     ]
                echo "Plot generated."
            }
        }
    }

    post {
        always {
            script {
                if (params.GIT_BRANCH == 'jenkins_test') {
                    // Archiving the generated files for future reference
                    archiveArtifacts artifacts: 'cppcheck-report.xml, cppcheck-results.csv, cppcheck-html/**', allowEmptyArchive: true
                } else {
                    // For other branches, do not archive cppcheck-results.csv
                    archiveArtifacts artifacts: 'cppcheck-report.xml, cppcheck-html/**', allowEmptyArchive: true
                }
            }
            echo "Build and analysis completed."

            // Publishing the HTML report
            publishHTML([
                reportDir: 'cppcheck-html',
                reportFiles: 'index.html',
                reportName: 'Cppcheck HTML Report',
                keepAll: true,
                alwaysLinkToLastBuild: true,
                allowMissing: false
            ])
            echo "HTML report published."
        }
    }
}
